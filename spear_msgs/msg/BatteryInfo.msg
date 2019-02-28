#
# Single battery info.
#

Header header

# Primary parameters.
float32 temperature             # [Kelvin]
float32 voltage                 # [Volt]
float32 current                 # [Ampere]
float32 average_power_10sec     # [Watt]      Average power consumption over the last 10 seconds.
float32 remaining_capacity_wh   # [Watt-hour] Will be increasing during charging.
float32 full_charge_capacity_wh # [Watt-hour] Predicted battery capacity when it is fully charged. Falls with aging.
float32 hours_to_full_charge    # [Hours]     Charging is expected to complete in this time. Zero if not charging.

# Status flags. These are mapped from bitfields defined in the relevant UAVCAN
# BatteryInfo type. ROS doesn't have bitfields, so they're listed here as bools.
bool in_use       # true = The battery is currently being used as a power supply
bool charging     # true = Charger is active
bool charged      # true = Charging complete, but charger is still active
bool temp_hot     # true = Battery is too hot
bool temp_cold    # true = Battery is too cold
bool overload     # true = Battery is violating the "safe operating area"
bool bad_battery  # true = This battery should not be used anymore
bool need_service # true = This battery requires maintenance
bool bms_error    # true = The battery management system has experienced an error

# State of Health (SOH) estimate, in percent.
# http://en.wikipedia.org/wiki/State_of_health
#
# If this equals 127 then the state of health is unknown.
uint8 state_of_health_pct

# Relative State of Charge (SOC) estimate, in percent.
# http://en.wikipedia.org/wiki/State_of_charge
uint8 state_of_charge_pct       # Percent of full charge [0 ->100].
uint8 state_of_charge_pct_stdev # SOC error standard deviation; use best guess if unknown.

# Battery identification.
# Model instance ID must be unique within the same battery model name.
# Model name is a human-readable string that normally should include the vendor name, model name, and chemistry
# type of this battery. This field should be assumed case-insensitive. Example: "Zubax Smart Battery v1.1 LiPo".
uint8 battery_id                        # Identifies the battery within this vehicle, e.g. 0 - primary battery
uint32 model_instance_id                # Set to zero if not applicable
string model_name                   # Battery model name
