import os

counter_file = os.path.join(os.path.dirname(__file__), "..", "build_counter.txt")
output_header = os.path.join(os.path.dirname(__file__), "..", "main/build", "build_counter.h")

# Read current value
try:
    with open(counter_file, "r") as f:
        count = int(f.read().strip())
except:
    count = 0

# Increment
count += 1

# Write back
with open(counter_file, "w") as f:
    f.write(str(count))

# Generate header
os.makedirs(os.path.dirname(output_header), exist_ok=True)
with open(output_header, "w") as f:
    f.write("#pragma once\n")
    f.write(f"#define BUILD_COUNTER {count}\n")

print(f"Build counter updated: File={output_header} Counter={count}")