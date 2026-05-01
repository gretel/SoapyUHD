// Copyright (c) 2026 SoapyUHD contributors
// SPDX-License-Identifier: GPL-3.0
//
// Unit tests for TypeHelpers.hpp free functions.
// Bare-stdlib harness; no external test framework.

#include "TypeHelpers.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

static int g_failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        std::cerr << "FAIL: " #cond " at " << __FILE__ << ":" << __LINE__ << "\n"; \
        ++g_failures; \
    } \
} while (0)

#define CHECK_NEAR(a, b, eps) do { \
    if (std::fabs((a) - (b)) > (eps)) { \
        std::cerr << "FAIL: |" #a " - " #b "| > " #eps \
                  << " (got " << (a) << " vs " << (b) << ")" \
                  << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
        ++g_failures; \
    } \
} while (0)

static void test_dict_kwargs_round_trip()
{
    SoapySDR::Kwargs kw;
    kw["driver"] = "uhd";
    kw["type"]   = "b200";
    kw["serial"] = "BADC10E";

    uhd::device_addr_t addr = kwargsToDict(kw);
    CHECK(addr.has_key("driver"));
    CHECK(addr.has_key("type"));
    CHECK(addr.has_key("serial"));
    CHECK(addr["driver"] == "uhd");

    SoapySDR::Kwargs kw2 = dictToKwargs(addr);
    CHECK(kw2.size() == 3);
    CHECK(kw2.at("driver") == "uhd");
    CHECK(kw2.at("type")   == "b200");
    CHECK(kw2.at("serial") == "BADC10E");
}

static void test_dict_kwargs_empty()
{
    SoapySDR::Kwargs kw;
    uhd::device_addr_t addr = kwargsToDict(kw);
    CHECK(addr.size() == 0);

    SoapySDR::Kwargs kw2 = dictToKwargs(addr);
    CHECK(kw2.empty());
}

static void test_meta_range_to_range_list_single()
{
    uhd::meta_range_t mr(0.0, 100.0, 1.0);
    SoapySDR::RangeList rl = metaRangeToRangeList(mr);
    CHECK(rl.size() == 1);
    CHECK_NEAR(rl[0].minimum(),   0.0, 1e-9);
    CHECK_NEAR(rl[0].maximum(), 100.0, 1e-9);
}

static void test_meta_range_to_range_list_multi()
{
    uhd::meta_range_t mr;
    mr.push_back(uhd::range_t(0.0, 10.0));
    mr.push_back(uhd::range_t(20.0, 30.0));
    SoapySDR::RangeList rl = metaRangeToRangeList(mr);
    CHECK(rl.size() == 2);
    CHECK_NEAR(rl[0].maximum(), 10.0, 1e-9);
    CHECK_NEAR(rl[1].minimum(), 20.0, 1e-9);
}

static void test_range_list_to_meta_range_round_trip()
{
    SoapySDR::RangeList rl;
    rl.push_back(SoapySDR::Range(1.0, 2.0));
    rl.push_back(SoapySDR::Range(5.0, 10.0));
    uhd::meta_range_t mr = rangeListToMetaRange(rl);
    CHECK(mr.size() == 2);
    CHECK_NEAR(mr[0].start(),  1.0, 1e-9);
    CHECK_NEAR(mr[1].stop(),  10.0, 1e-9);
}

static void test_range_list_to_meta_range_empty_yields_zero()
{
    SoapySDR::RangeList rl;
    uhd::meta_range_t mr = rangeListToMetaRange(rl);
    CHECK(mr.size() == 1);
    CHECK_NEAR(mr[0].start(), 0.0, 1e-9);
    CHECK_NEAR(mr[0].stop(),  0.0, 1e-9);
}

static void test_meta_range_to_range_top_level()
{
    uhd::meta_range_t mr(-50.0, 50.0, 0.5);
    SoapySDR::Range r = metaRangeToRange(mr);
    CHECK_NEAR(r.minimum(), -50.0, 1e-9);
    CHECK_NEAR(r.maximum(),  50.0, 1e-9);
}

static void test_number_list_to_meta_range_empty_yields_zero()
{
    std::vector<double> v;
    uhd::meta_range_t mr = numberListToMetaRange(v);
    CHECK(mr.size() == 1);
    CHECK_NEAR(mr[0].start(), 0.0, 1e-9);
}

static void test_meta_range_to_numeric_list_single_returns_bounds()
{
    uhd::meta_range_t mr(10.0, 20.0);
    std::vector<double> v = metaRangeToNumericList(mr);
    CHECK(v.size() == 2);
    CHECK_NEAR(v[0], 10.0, 1e-9);
    CHECK_NEAR(v[1], 20.0, 1e-9);
}

static void test_meta_range_to_numeric_list_multi_returns_starts()
{
    uhd::meta_range_t mr;
    mr.push_back(uhd::range_t(1.0));
    mr.push_back(uhd::range_t(2.0));
    mr.push_back(uhd::range_t(3.0));
    std::vector<double> v = metaRangeToNumericList(mr);
    CHECK(v.size() == 3);
    CHECK_NEAR(v[0], 1.0, 1e-9);
    CHECK_NEAR(v[2], 3.0, 1e-9);
}

static void test_range_to_meta_range_default_step()
{
    SoapySDR::Range r(0.0, 100.0);
    uhd::meta_range_t mr = rangeToMetaRange(r);
    CHECK_NEAR(mr.start(),   0.0, 1e-9);
    CHECK_NEAR(mr.stop(),  100.0, 1e-9);
}

static void test_sensor_arginfo_round_trip_bool()
{
    uhd::sensor_value_t s("locked", true, "true", "false");
    SoapySDR::ArgInfo ai = sensorToArgInfo(s, "locked");
    CHECK(ai.key  == "locked");
    CHECK(ai.type == SoapySDR::ArgInfo::BOOL);

    uhd::sensor_value_t s2 = argInfoToSensor(ai, "true");
    CHECK(s2.to_bool() == true);
}

static void test_sensor_arginfo_round_trip_int()
{
    uhd::sensor_value_t s("count", 42, "items");
    SoapySDR::ArgInfo ai = sensorToArgInfo(s, "count");
    CHECK(ai.type == SoapySDR::ArgInfo::INT);

    uhd::sensor_value_t s2 = argInfoToSensor(ai, "42");
    CHECK(s2.to_int() == 42);
}

static void test_sensor_arginfo_round_trip_real()
{
    uhd::sensor_value_t s("temp", 25.5, "C");
    SoapySDR::ArgInfo ai = sensorToArgInfo(s, "temp");
    CHECK(ai.type == SoapySDR::ArgInfo::FLOAT);

    uhd::sensor_value_t s2 = argInfoToSensor(ai, "25.5");
    CHECK_NEAR(s2.to_real(), 25.5, 1e-9);
}

static void test_sensor_arginfo_round_trip_string()
{
    uhd::sensor_value_t s("status", std::string("active"), "");
    SoapySDR::ArgInfo ai = sensorToArgInfo(s, "status");
    CHECK(ai.type == SoapySDR::ArgInfo::STRING);

    uhd::sensor_value_t s2 = argInfoToSensor(ai, "active");
    CHECK(s2.value == "active");
}

int main()
{
    test_dict_kwargs_round_trip();
    test_dict_kwargs_empty();
    test_meta_range_to_range_list_single();
    test_meta_range_to_range_list_multi();
    test_range_list_to_meta_range_round_trip();
    test_range_list_to_meta_range_empty_yields_zero();
    test_meta_range_to_range_top_level();
    test_number_list_to_meta_range_empty_yields_zero();
    test_meta_range_to_numeric_list_single_returns_bounds();
    test_meta_range_to_numeric_list_multi_returns_starts();
    test_range_to_meta_range_default_step();
    test_sensor_arginfo_round_trip_bool();
    test_sensor_arginfo_round_trip_int();
    test_sensor_arginfo_round_trip_real();
    test_sensor_arginfo_round_trip_string();

    if (g_failures == 0)
    {
        std::cout << "All tests passed.\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) FAILED.\n";
    return 1;
}
