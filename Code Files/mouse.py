from evdev import InputDevice, categorize, ecodes

# Find the event file corresponding to the USB mouse
event_file = '/dev/input/event1'  # Replace X with the event number of your mouse

# CPI of the mouse (replace this with the CPI of your mouse)
mouse_cpi = 1000

try:
    mouse = InputDevice(event_file)
except FileNotFoundError:
    print("Mouse not found. Make sure it's connected and check the event file.")
    exit()

# Calculate conversion factor from counts to centimeters
conversion_factor = (2.54 / mouse_cpi)*2

# Function to parse the event for mouse movements
def parse_event(event):
    if event.type == ecodes.EV_REL:
        # Check for relative motion events (mouse movements)
        if event.code == ecodes.REL_X:
            return event.value, 0
        elif event.code == ecodes.REL_Y:
            return 0, event.value
    return 0, 0

print("Reading mouse events. Press Ctrl+C to exit.")

try:
    x_displacement_cm = 0
    y_displacement_cm = 0

    for event in mouse.read_loop():
        x_delta, y_delta = parse_event(event)
        x_displacement_cm += x_delta * conversion_factor
        y_displacement_cm += y_delta * conversion_factor
        print(f"X displacement: {x_displacement_cm:.2f} cm, Y displacement: {y_displacement_cm:.2f} cm")

except KeyboardInterrupt:
    print("\nExiting...")