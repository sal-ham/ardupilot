#!/usr/bin/env python3
"""Run the real RPLIDAR driver against a small deterministic UART/backend fixture.

The three driver files are copied without modification. The fixture replaces
only the hardware/ArduPilot environment, so this is not a full firmware or SITL
test. Requires a C++17 compiler; use --cxx 'path/to/zig.exe c++' on Windows.
"""
import argparse
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile

STUBS = {
    'AP_HAL/AP_HAL_Boards.h': '''#pragma once
#define BOARD_FLASH_SIZE 2048
#define CONFIG_HAL_BOARD 1
#define HAL_BOARD_SITL 1
''',
    'GCS_MAVLink/GCS_config.h': '#pragma once\n#define HAL_GCS_ENABLED 0\n',
    'AP_RangeFinder/AP_RangeFinder_config.h': '#pragma once\n#define AP_RANGEFINDER_ENABLED 0\n',
    'GCS_MAVLink/GCS.h': '''#pragma once
#define MAV_SEVERITY_INFO 6
#define MAV_SEVERITY_WARNING 4
template<typename... Args> inline void fixture_text(int, const char*, Args...) {}
#define GCS_SEND_TEXT(...) fixture_text(__VA_ARGS__)
''',
    'AP_InternalError/AP_InternalError.h': '''#pragma once
namespace AP_InternalError { enum class error_t { flow_of_control }; }
#define INTERNAL_ERROR(...) (++fixture_errors)
''',
    'AP_HAL/AP_HAL.h': '#pragma once\n#include "fixture.h"\n',
    'AP_Proximity_Backend_Serial.h': '#pragma once\n#include "fixture.h"\n',
    'fixture.h': r'''#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <new>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <vector>
#define PACKED __attribute__((packed))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define DEFINE_BYTE_ARRAY_METHODS \
    uint8_t &operator[](size_t n) { return reinterpret_cast<uint8_t*>(this)[n]; } \
    const uint8_t &operator[](size_t n) const { return reinterpret_cast<const uint8_t*>(this)[n]; }
inline unsigned fixture_errors = 0;
inline uint32_t fixture_us = 4001000;
inline uint32_t micros_step = 0;
inline float wrap_360(float a) { a = std::fmod(a, 360.0f); return a < 0 ? a + 360.0f : a; }
namespace AP_HAL {
struct HAL {};
inline uint32_t millis() { return fixture_us / 1000; }
inline uint32_t micros() { uint32_t t = fixture_us; fixture_us += micros_step; return t; }
class UARTDriver {
public:
    std::deque<uint8_t> rx;
    std::vector<uint8_t> tx;
    bool fail_read = false;
    uint32_t available() const { return static_cast<uint32_t>(rx.size()); }
    ssize_t read(uint8_t *dst, uint16_t count) {
        if (fail_read) { return -1; }
        uint16_t n = std::min<uint16_t>(count, static_cast<uint16_t>(rx.size()));
        for (uint16_t i = 0; i < n; ++i) { dst[i] = rx.front(); rx.pop_front(); }
        return n;
    }
    void write(const uint8_t *src, size_t n) { tx.insert(tx.end(), src, src+n); }
    void inject(const std::vector<uint8_t> &bytes) { rx.insert(rx.end(), bytes.begin(), bytes.end()); }
};
}
class AP_Proximity_Boundary_3D {
public:
    class Face {
    public:
        uint8_t layer = UINT8_MAX, sector = UINT8_MAX;
        Face() = default;
        Face(uint8_t l, uint8_t s) : layer(l), sector(s) {}
        bool valid() const { return layer < 5 && sector < 8; }
        bool operator==(const Face &f) const { return layer == f.layer && sector == f.sector; }
        bool operator!=(const Face &f) const { return !(*this == f); }
    };
    struct Reading { bool valid = false; float angle = 0, distance = 0; };
    std::array<Reading, 8> readings{};
    Face get_face(float angle) const { return Face(2, static_cast<uint8_t>(wrap_360(angle + 22.5f) / 45.0f)); }
    void set_face_attributes(const Face &f, float a, float d, uint8_t) {
        if (!f.valid()) { throw std::runtime_error("invalid boundary face"); }
        readings[f.sector] = {true, a, d};
    }
    void reset_face(const Face &f, uint8_t) { if (f.valid()) { readings[f.sector].valid = false; } }
};
struct AP_Proximity_Params {
    int orientation = 0;
    float yaw_correction = 0, min_m = 0, max_m = 0;
    std::array<float, 4> ignore_angle_deg{}, ignore_width_deg{};
};
class AP_Proximity {
public:
    enum class Status { NoData, Good };
    struct Proximity_State { uint8_t instance = 0; Status status = Status::NoData; };
    struct Point { float angle, distance; };
    AP_HAL::UARTDriver uart;
    AP_Proximity_Boundary_3D boundary;
    std::vector<Point> database;
};
class AP_Proximity_Backend_Serial {
public:
    AP_Proximity_Backend_Serial(AP_Proximity &f, AP_Proximity::Proximity_State &s,
                                AP_Proximity_Params &p, uint8_t)
        : _uart(&f.uart), frontend(f), state(s), params(p) {}
    virtual ~AP_Proximity_Backend_Serial() = default;
    virtual void update() = 0;
    virtual float distance_min() const = 0;
    virtual float distance_max() const = 0;
protected:
    AP_HAL::UARTDriver *_uart;
    AP_Proximity &frontend;
    AP_Proximity::Proximity_State &state;
    AP_Proximity_Params &params;
    void set_status(AP_Proximity::Status s) { state.status = s; }
    bool ignore_reading(float a, float d) const {
        if ((params.min_m != 0 && d < params.min_m) || (params.max_m != 0 && d > params.max_m)) { return true; }
        for (size_t i = 0; i < 4; ++i) {
            if (params.ignore_width_deg[i] != 0 && std::abs(a - params.ignore_angle_deg[i]) <= params.ignore_width_deg[i]/2) { return true; }
        }
        return false;
    }
    void database_push(float a, float d) { frontend.database.push_back({a,d}); }
};
'''
}

TEST_CPP = r'''
#include "fixture.h"
#include "driver/AP_Proximity_RPLidarA2.h"
#include "driver/AP_Proximity_RPLidarA2.cpp"
const AP_HAL::HAL fixture_hal{};
const AP_HAL::HAL &hal = fixture_hal;
static void check(bool ok, const char *message) { if (!ok) { throw std::runtime_error(message); } }
static bool near(float a, float b) { return std::abs(a-b) < 0.025f; }
struct Rig {
    AP_Proximity frontend;
    AP_Proximity::Proximity_State state;
    AP_Proximity_Params params;
    alignas(AP_Proximity_RPLidarA2) unsigned char storage[sizeof(AP_Proximity_RPLidarA2)]{};
    AP_Proximity_RPLidarA2 &driver;
    Rig() : driver(*new(storage) AP_Proximity_RPLidarA2(frontend,state,params,0)) { fixture_us=4001000; micros_step=0; fixture_errors=0; }
    ~Rig() { driver.~AP_Proximity_RPLidarA2(); }
    void inject(const std::vector<uint8_t> &v) { frontend.uart.inject(v); fixture_us += 1000; driver.update(); }
    void identify(uint8_t model) {
        driver.update();
        std::vector<uint8_t> info{0xA5,0x5A,0x14,0,0,0,0x04};
        info.resize(27); info[7]=model; info[8]=1; info[9]=2; info[10]=18;
        inject(info);
    }
    void front_only() {
        params.ignore_angle_deg = {70,180,290,0};
        params.ignore_width_deg = {110,110,110,0};
    }
    void s2() {
        identify(0x71);
        const std::vector<uint8_t> expected{0xA5,0x82,0x05,0,0,0,0,0,0x22};
        const auto &tx = frontend.uart.tx;
        check(tx.size() >= expected.size(), "missing S2 scan request");
        check(std::equal(expected.begin(),expected.end(),tx.end()-expected.size()), "S2 scan command/checksum mismatch");
        inject({0xA5,0x5A,0x54,0,0,0x40,0x85});
    }
};
#if AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED
static std::vector<uint8_t> dense(float angle, uint16_t mm, bool encoder_reset=false, int only_index=-1) {
    std::vector<uint8_t> b(84,0);
    uint16_t start = static_cast<uint16_t>(std::lround(angle*64.0f));
    if (encoder_reset) { start |= 0x8000; }
    b[2]=start&255; b[3]=start>>8;
    for (int i=0; i<40; ++i) {
        uint16_t d = (only_index < 0 || i == only_index) ? mm : 0;
        b[4+2*i]=d&255; b[5+2*i]=d>>8;
    }
    uint8_t sum=0; for (size_t i=2;i<b.size();++i) { sum^=b[i]; }
    b[0]=0xA0|(sum&15); b[1]=0x50|(sum>>4);
    return b;
}
static void scan(Rig &r, uint16_t mm, unsigned revolutions=2) {
    for (unsigned n=0;n<revolutions;++n) {
        for (unsigned i=0;i<80;++i) { r.inject(dense(i*4.5f,mm)); }
    }
    r.inject(dense(0,mm));
}
#endif
static std::vector<uint8_t> legacy(float angle, float meters) {
    uint16_t a=static_cast<uint16_t>(angle*64); uint16_t d=static_cast<uint16_t>(meters*4000);
    return {0x21,static_cast<uint8_t>((a<<1)|1),static_cast<uint8_t>(a>>7),static_cast<uint8_t>(d&255),static_cast<uint8_t>(d>>8)};
}
static void a2_legacy() {
    Rig r; r.identify(0x28);
    auto &tx=r.frontend.uart.tx;
    check(tx.size()>=2 && tx[tx.size()-2]==0xA5 && tx.back()==0x20,"A2 standard scan request changed");
    r.inject({0xA5,0x5A,0x05,0,0,0x40,0x81});
    r.inject(legacy(0,4)); r.inject(legacy(45,5));
    check(r.frontend.boundary.readings[0].valid && near(r.frontend.boundary.readings[0].distance,4),"A2 boundary regression");
    check(fixture_errors==0,"A2 internal error");
}
#if AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED
static void full_scan() {
    Rig r; r.s2(); scan(r,5000);
    for (const auto &f:r.frontend.boundary.readings) { check(f.valid && near(f.distance,5),"full scan sector missing/wrong distance"); }
    check(r.state.status==AP_Proximity::Status::Good,"healthy S2 reports NoData");
    check(fixture_errors==0,"full scan internal error");
}
static void front_mask_refresh() {
    Rig r; r.front_only(); r.s2(); scan(r,1500);
    check(r.frontend.boundary.readings[0].valid && near(r.frontend.boundary.readings[0].distance,1.5f),"front-only reading is not published");
    for (size_t i=1;i<8;++i) { check(!r.frontend.boundary.readings[i].valid,"ignored side/rear sector published"); }
    scan(r,6000);
    check(near(r.frontend.boundary.readings[0].distance,6),"retreating obstacle retains old minimum");
    scan(r,0);
    check(!r.frontend.boundary.readings[0].valid,"removed obstacle leaves stale front reading");
}
static void encoder_reset_flag() {
    Rig r; r.front_only(); r.s2(); r.inject(dense(0,3000,true)); r.inject(dense(4.5f,0));
    check(!r.frontend.database.empty(),"encoder-reset bit interpreted as angle");
    check(near(r.frontend.database.front().angle,0),"encoder-reset angle is not zero");
}
static void measured_angle_step() {
    Rig r; r.front_only(); r.s2();
    r.inject(dense(12,1000,false,25)); r.inject(dense(18,0)); r.inject(dense(24,0));
    check(r.frontend.database.empty(),"fixed angle step admits a sample outside the 30 degree front view");
}
static void fragmented_and_corrupt_stream() {
    Rig r; r.s2(); auto b=dense(0,2000); b[0]^=1; r.inject(b);
    check(r.frontend.database.empty(),"corrupted packet accepted");
    r.inject({0x42,0x19,0xFF,0x00,0x80});
    b=dense(4.5f,2500);
    for (size_t i=0;i<b.size();i+=7) {
        size_t end=std::min(i+7,b.size()); r.inject(std::vector<uint8_t>(b.begin()+i,b.begin()+end));
    }
    r.inject(dense(9,2500)); r.inject(dense(13.5f,2500));
    check(!r.frontend.database.empty() && near(r.frontend.database.back().distance,2.5f),"stream did not recover after noise/fragmentation");
    check(fixture_errors==0,"corrupt stream internal error");
}
static void orientation_and_timeout() {
    Rig r; r.params.orientation=1; r.params.yaw_correction=90; r.s2(); scan(r,4000);
    check(r.frontend.boundary.readings[2].valid,"orientation/yaw correction lost sector");
    fixture_us+=201000; r.driver.update();
    check(r.state.status==AP_Proximity::Status::NoData,"silence does not mark S2 NoData");
}
static void invalid_ranges() {
    Rig r; r.s2(); r.inject(dense(0,49)); r.inject(dense(4.5f,51000)); r.inject(dense(9,0));
    check(r.frontend.database.empty(),"distance outside driver limits accepted");
}
static void split_descriptor_and_capsule() {
    Rig r; r.identify(0x71); auto b=dense(0,4000);
    std::vector<uint8_t> combined{0xA5,0x5A,0x54,0,0,0x40,0x85};
    combined.insert(combined.end(),b.begin(),b.end()); r.inject(combined);
    r.inject(dense(4.5f,4000)); r.inject(dense(9,4000));
    check(!r.frontend.database.empty(),"scan bytes following descriptor were discarded");
}
static void receive_budget() {
    Rig r; r.s2(); for (int i=0;i<32;++i) { r.frontend.uart.inject(dense(i*4.5f,4000)); }
    micros_step=50;
    for(int i=0;i<200 && !r.frontend.uart.rx.empty();++i) { r.driver.update(); }
    micros_step=0; r.driver.update();
    check(r.frontend.uart.rx.empty(),"bounded processing cannot drain a queued stream");
    check(fixture_errors==0,"receive budget internal error");
}
static void serial_read_error() {
    Rig r; r.s2();
    r.frontend.uart.fail_read=true;
    r.inject(dense(0,4000));
    check(r.frontend.uart.rx.size()==84 && r.frontend.database.empty(),"failed serial read consumed data");
    r.frontend.uart.fail_read=false;
    r.driver.update();
    r.inject(dense(4.5f,4000));
    check(r.frontend.uart.rx.empty() && !r.frontend.database.empty(),"serial read error damaged the receive buffer");
    check(near(r.frontend.database.front().distance,4),"serial read recovery produced a wrong distance");
    check(fixture_errors==0,"serial read error internal error");
}
#else
static void s2_disabled() {
    Rig r; r.identify(0x71);
    check(r.frontend.uart.tx.size()==2,"S2 used the legacy scan command when Express is disabled");
}
#endif
int main(int argc,char **argv) {
    struct Test { const char *name; void(*run)(); };
    const Test tests[] = {
        {"a2_legacy",a2_legacy},
#if AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED
        {"full_scan",full_scan}, {"front_mask_refresh",front_mask_refresh},
        {"encoder_reset_flag",encoder_reset_flag}, {"measured_angle_step",measured_angle_step},
        {"fragmented_and_corrupt_stream",fragmented_and_corrupt_stream},
        {"orientation_and_timeout",orientation_and_timeout}, {"invalid_ranges",invalid_ranges},
        {"split_descriptor_and_capsule",split_descriptor_and_capsule}, {"receive_budget",receive_budget},
        {"serial_read_error",serial_read_error},
#else
        {"s2_disabled",s2_disabled},
#endif
    };
    unsigned failures=0;
    for(const auto &t:tests) {
        if(argc>1 && std::string(argv[1])!=t.name) { continue; }
        try { t.run(); std::printf("PASS %s\n",t.name); }
        catch(const std::exception &e) { ++failures; std::printf("FAIL %s: %s\n",t.name,e.what()); }
    }
    return failures?1:0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--cxx', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--case')
    parser.add_argument('--work-dir', type=Path, help='Optional directory for compiler temporary files')
    args = parser.parse_args()
    command = shlex.split(args.cxx, posix=os.name != 'nt')
    if os.name == 'nt':
        # Match ARM GCC's packed wire-bitfield layout instead of the Windows ABI.
        command += ['-mno-ms-bitfields']
    if args.work_dir:
        args.work_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rplidar-s2-', dir=args.work_dir) as directory:
        work = Path(directory)
        compiler_env = os.environ.copy()
        compiler_env['TEMP'] = compiler_env['TMP'] = str(work.resolve())
        for name, contents in STUBS.items():
            target = work/name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(contents, encoding='utf-8')
        (work/'driver').mkdir()
        for name in ['AP_Proximity_RPLidarA2.cpp', 'AP_Proximity_RPLidarA2.h', 'AP_Proximity_config.h']:
            shutil.copyfile(args.root/'libraries'/'AP_Proximity'/name, work/'driver'/name)
        (work/'test.cpp').write_text(TEST_CPP, encoding='utf-8')
        for express in [1,0]:
            output = work/('test-%s%s' % (express, '.exe' if os.name == 'nt' else ''))
            print('Compiling real driver, EXPRESSSCAN=%s' % express, flush=True)
            subprocess.run(command + ['-std=gnu++17', '-Wall', '-Wextra', '-Werror',
                '-I'+str(work),
                '-DAP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED='+str(express),
                str(work/'test.cpp'), '-o',str(output)], check=True, env=compiler_env)
            result = subprocess.run([str(output)] + ([args.case] if args.case else []))
            if result.returncode:
                raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
