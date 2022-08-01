#include <algorithm>
#include <unity.h>
#include <TMCStepper.h>
#include "Mocks.h"

static constexpr uint8_t registerAddress = 0x01;

struct test_fixture_get_gstat {
    test_fixture_get_gstat() {
        SPI.responses.emplace_back(0);
        SPI.responses.emplace_back(0x3);
        expectedCommands.emplace_back(registerAddress, 0); // Read
        expectedCommands.emplace_back(registerAddress, 0); // Read
    }

    SPIClass SPI{};
    TMC2130Stepper driver{SPI, 0, 1.0};
    std::deque<SPIClass::Payload> expectedCommands;
};

void test_TMC2130_get_reset() {
    test_fixture_get_gstat test{};

    auto bit = test.driver.reset();

    TEST_ASSERT_TRUE_MESSAGE(test.expectedCommands == test.SPI.sentCommands, test.SPI);
    TEST_ASSERT_TRUE(bit);
}

void test_TMC2130_get_drv_err() {
    test_fixture_get_gstat test{};

    auto bit = test.driver.drv_err();

    TEST_ASSERT_TRUE_MESSAGE(test.expectedCommands == test.SPI.sentCommands, test.SPI);
    TEST_ASSERT_TRUE(bit);
}
