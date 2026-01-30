#include <cstdint>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief How often the node publishes its data
 */
constexpr std::chrono::duration PUBLISH_PERIOD = 1s;

/**
 * @brief Value of the sense resistor for the MAX17263, in ohms
 *
 * Probably between 0.010 and 0.020
 */
constexpr double R_SENSE = 0.010;

/**
 * @brief Number of LEDs to configure the chip to use. Use UINT8_MAX for
 * auto-detection.
 */
constexpr uint8_t LED_COUNT = 5;

/**
 * @brief Designed manimum caacity of the battery
 * 
 */
constexpr uint32_t DESIGN_CAPACITY_mAh = 4200;
