import Jetson.GPIO as GPIO
from enum import Enum
import time

class GPIOControl:
    class NumberingMode(Enum):
        BOARD = GPIO.BOARD
        BCM = GPIO.BCM
        CVM = GPIO.CVM
        TEGRA = GPIO.TEGRA_SOC
    
    class Direction(Enum):
        IN = GPIO.IN
        OUT = GPIO.OUT

    class RobotState(Enum):
        # Name          = Pulse width (microseconds)
        DOCKED          = 1415    # Breath, Red
        IDLE            = 1545    # Breath Slow (Colour 1)
        TO_PICKUP_STATION = 1345    # Light Chase (Red)
        AT_PICKUP_STATION = 1885    # Red
        ITEM_PICKED_UP  = 1065    # Confetti
        TO_DELIVERY     = 1355    # Light Chase (Blue)
        AT_DELIVERY     = 1935    # Blue
        ITEM_DELIVERED  = 1225    # Twinkles, Rainbow Palette
        RETURNING       = 1365    # Light Chase (Gray)
        CHARGING        = 1515    # Heartbeat Slow (Colour 1 - Green)
        CRITICAL_ERROR  = 1445    # Strobe (Red)

    def __init__(self, mode = NumberingMode.BOARD):
        GPIO.setmode(mode.value)
        self.mode = mode
        self.pin_states = {}
        self.pin_directions = {}
        self.pwm_objects = {}
        self.current_robot_state_pin = None

    def cleanup(self):
        GPIO.cleanup()

    def print_num_mode(self):
        if self.mode == self.NumberingMode.BCM:
            print("BCM numbering mode set.")
        elif self.mode == self.NumberingMode.CVM:
            print("CVM numbering mode set.")
        elif self.mode == self.NumberingMode.TEGRA:
            print("TEGRA_SOC numbering mode set.")
        else:
            print("BOARD numbering mode set.")

    def setup_pin(self, pin, direction, initial = GPIO.LOW):
        if pin in self.pin_directions:
            print(f"Warning: Pin {pin} is already configured.")
            return

        GPIO.setup(pin, direction.value)
        self.pin_directions[pin] = direction

        if direction == self.Direction.OUT:
            GPIO.output(pin, initial)
            self.pin_states[pin] = bool(initial)

    def _validate_output_pin(self, pin):
        if pin not in self.pin_directions:
            print(f"Warning: Pin {pin} has not been configured.")
            return False
        if self.pin_directions[pin] == self.Direction.IN:
            print(f"Warning: Pin {pin} has been configured as an input pin.")
            return False
        return True
    
    def set_high(self, pin):
        if not self._validate_output_pin(pin):
            return
        GPIO.output(pin, GPIO.HIGH)
        self.pin_states[pin] = True

    def set_low(self, pin):
        if not self._validate_output_pin(pin):
            return
        GPIO.output(pin, GPIO.LOW)
        self.pin_states[pin] = False

    def toggle(self, pin):
        if not self._validate_output_pin(pin):
            return
        if self.pin_states[pin] == False:
            self.set_high(pin)
        else:
            self.set_low(pin)

    def read_pin_state(self, pin):
        if pin not in self.pin_directions:
            print(f"Error: Pin {pin} has not been configured.")
            return False

        if self.pin_directions[pin] == self.Direction.IN:
            return GPIO.input(pin)

        return self.pin_states.get(pin, False)

    def pin_function(self, pin):
        if pin not in self.pin_directions:
            print(f"Error: Pin {pin} has not been configured.")
            return

        direction = self.pin_directions[pin]
        print(f"Pin {pin} is an {'input' if direction == self.Direction.IN else 'output'} pin.")

    def setup_pwm(self, pin, freq_hz, initial_duty_cycle = 0):
        if pin in self.pin_directions:
            print(f"Warning: Pin {pin} is already configured as GPIO. Reconfigure for PWM.")
            return
       
        if pin in self.pwm_objects:
            print(f"Warning: PWM already configured on pin {pin}.")
            return
       
        try:
            p = GPIO.PWM(pin, freq_hz)
            p.start(initial_duty_cycle)
            self.pwm_objects[pin] = p
            print(f"PWM set up on pin {pin} at {freq_hz} Hz with {initial_duty_cycle}% duty cycle.")
        except Exception as e:
            print(f"Error setting up PWM on pin {pin}: {e}. Ensure pinmux is configured.")

    def set_pwm_pulse_width(self, pin, pulse_width_us, freq_hz = 50):
        period_us = 1_000_000 / freq_hz
        duty_cycle = (pulse_width_us / period_us) * 100

        if pin not in self.pwm_objects:
            self.setup_pwm(pin, freq_hz, duty_cycle)
        else:
            self.pwm_objects[pin].ChangeDutyCycle(duty_cycle)
            print(f"PWM on pin {pin} set to {pulse_width_us}us pulse width ({duty_cycle:.2f}% duty cycle.)")

    def stop_pwm(self, pin):
        if pin in self.pwm_objects:
            self.pwm_objects[pin].stop()
            del self.pwm_objects[pin]
            print(f"PWM stopped on pin {pin}.")
        else:
            print(f"Warning: No PWM running on pin {pin}.")
     
    def set_robot_status_pattern(self, pin, robot_state, freq_hz = 50):
        if not isinstance(robot_state, self.RobotState):
            print("Error: Invalid robot state provided. Must be a member of GPIOControl.RobotState.")
            return
     
        pulse_width_us = robot_state.value
        self.set_pwm_pulse_width(pin, pulse_width_us, freq_hz)
        self.current_robot_state_pin = pin

    def clear_robot_status_pattern(self):
        if self.current_robot_state_pin is not None:
            self.stop_pwm(self.current_robot_state_pin)
            self.current_robot_state_pin = None
        else:
            print("No robot status pattern is currently active")

# --- Example Usage ---
if __name__ == "__main__":
    # It's crucial to know which physical pin number you want to use.
    # For example, let's use BOARD pin 7 (which is GPIO_PZ00 on Jetson Nano).
    # Please replace '7' with an actual available GPIO pin on your Jetson board.
    # Refer to your specific Jetson board's pinout diagram.
    GPIO_PIN = 7 

    gpio_control = None # Initialize to None for cleanup in finally block
    try:
        # Initialize the GPIOControl class (defaulting to BOARD numbering mode)
        gpio_control = GPIOControl()
        gpio_control.print_num_mode()

        # Set up the chosen pin as an output pin
        print(f"\nSetting up pin {GPIO_PIN} as an OUTPUT pin...")
        gpio_control.setup_pin(GPIO_PIN, GPIOControl.Direction.OUT)

        # Set the pin high
        print(f"Setting pin {GPIO_PIN} HIGH...")
        gpio_control.set_high(GPIO_PIN)
        time.sleep(2) # Keep it high for 2 seconds

        # Set the pin low
        print(f"Setting pin {GPIO_PIN} LOW...")
        gpio_control.set_low(GPIO_PIN)
        time.sleep(2) # Keep it low for 2 seconds

        # Toggle the pin
        print(f"Toggling pin {GPIO_PIN} (should go HIGH)...")
        gpio_control.toggle(GPIO_PIN)
        time.sleep(1)

        print(f"Toggling pin {GPIO_PIN} again (should go LOW)...")
        gpio_control.toggle(GPIO_PIN)
        time.sleep(1)

        # Read the current state of the pin (will return the last set state for output pins)
        current_state = gpio_control.read_pin_state(GPIO_PIN)
        print(f"Current state of pin {GPIO_PIN}: {'HIGH' if current_state else 'LOW'}")

        # Demonstrate pin function
        gpio_control.pin_function(GPIO_PIN)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Clean up GPIO settings when done
        if gpio_control:
            print("\nCleaning up GPIO...")
            gpio_control.cleanup()