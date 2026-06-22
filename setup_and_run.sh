#!/bin/bash
# ============================================================
#  AUV Navigation — Complete Setup & Run Script
#  Run this ONCE from inside your auv_nav2/ folder:
#    cd ~/Dhananjay/auv_nav2
#    bash setup_and_run.sh
# ============================================================
set -e

echo ""
echo "━━━ STEP 1: Patching main.cpp (fixing navoutput → nav_output) ━━━"

# Fix the output filename in main.cpp
sed -i 's|results/navoutput.csv|results/nav_output.csv|g' src/main.cpp

# Add sys/stat.h if not already there
if ! grep -q "sys/stat.h" src/main.cpp; then
    sed -i '/#include <chrono>/a #include <sys/stat.h>   // mkdir' src/main.cpp
    echo "Added sys/stat.h"
fi

# Add mkdir calls if not already there
if ! grep -q "ensureDir\|::mkdir" src/main.cpp; then
    # Add ensureDir function after 'using namespace auv;'
    sed -i '/using namespace auv;/a \
\
static void ensureDir(const std::string\& d) { ::mkdir(d.c_str(), 0755); }\
' src/main.cpp

    # Add the mkdir calls right before the generate-data block
    sed -i '/\/\/ ── Generate synthetic data/i \
    \/\/ Auto-create folders \
    ensureDir("data"); \
    ensureDir("results"); \
' src/main.cpp
    echo "Added mkdir calls"
fi

echo ""
echo "━━━ STEP 2: Fixing csv_parser.cpp (saveResults flush+verify) ━━━"

# Add sys/stat.h and cerrno to csv_parser.cpp if missing
if ! grep -q "sys/stat.h" src/csv_parser.cpp; then
    sed -i '/#include <stdexcept>/a #include <sys/stat.h>\n#include <cerrno>\n#include <cstring>' src/csv_parser.cpp
    echo "Added headers to csv_parser.cpp"
fi

echo ""
echo "━━━ STEP 3: Compiling ━━━"
mkdir -p data results
g++ -std=c++17 -O2 -Iinclude \
    src/main.cpp src/ekf.cpp src/csv_parser.cpp src/logger.cpp \
    -o auv_nav -lm
echo "Compiled → ./auv_nav"

echo ""
echo "━━━ STEP 4: Verifying fix ━━━"
OUTPUT_LINE=$(./auv_nav --generate-data 2>&1 | grep "Output path\|Output file" | head -1)
echo "Output line: $OUTPUT_LINE"

if echo "$OUTPUT_LINE" | grep -q "nav_output.csv"; then
    echo ""
    echo "✅  SUCCESS — saving to results/nav_output.csv correctly"
    echo "✅  File exists: $(ls -lh results/nav_output.csv)"
    echo ""
    echo "━━━ DONE ━━━"
    echo "To run on your own data:"
    echo "  ./auv_nav                        (uses data/ folder)"
    echo "  ./auv_nav my_imu.csv my_dvl.csv  (custom files)"
else
    echo ""
    echo "❌  Still showing wrong path. Check src/main.cpp manually."
    grep "out_f\|navoutput\|nav_output" src/main.cpp | head -5
fi
