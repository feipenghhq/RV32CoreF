#!/usr/bin/sh

# Usage:
# gen_summary <TEST NAME> <Search file Pattern> <Output log file>

name=$1
search_file=$2
output_file=$3

passed=`cat $search_file | grep PASS -c`
failed=`cat $search_file | grep FAIL -c`
timeout=`cat $search_file | grep TIMEOUT -c`
not_passed=`expr $failed + $timeout`

passed_test=`cat $search_file | grep PASS | grep "TEST NAME: [a-z0-9\-]*" -o | sed "s/TEST NAME: //"`
failed_test=`cat $search_file | grep FAIL | grep "TEST NAME: [a-z0-9\-]*" -o | sed "s/TEST NAME: //"`
timeout_test=`cat $search_file | grep TIMEOUT | grep "TEST NAME: [a-z0-9\-]*" -o | sed "s/TEST NAME: //"`

rm -f $output_file

echo "\n\n"
echo "-------- Test Complete --------" | tee -a $output_file

if [ "$not_passed" -eq 0 ]; then
echo ""                                 | tee -a $output_file
echo "" | tee                           | tee -a $output_file
echo "      ____  ___   __________  "   | tee -a $output_file
echo "     / __ \/   | / ___/ ___/  "   | tee -a $output_file
echo "    / /_/ / /| | \__ \\__ \   "   | tee -a $output_file
echo "   / ____/ ___ |___/ /__/ /   "   | tee -a $output_file
echo "  /_/   /_/  |_/____/____/    "   | tee -a $output_file
echo ""                                 | tee -a $output_file
echo ""                                 | tee -a $output_file
fi

if [ "$not_passed" -ne 0 ]; then
echo ""                                 | tee -a $output_file
echo ""                                 | tee -a $output_file
echo "    _________    ______   "       | tee -a $output_file
echo "   / ____/   |  /  _/ /   "       | tee -a $output_file
echo "  / /_  / /| |  / // /    "       | tee -a $output_file
echo " / __/ / ___ |_/ // /___  "       | tee -a $output_file
echo "/_/   /_/  |_/___/_____/  "       | tee -a $output_file
echo "                          "       | tee -a $output_file
echo ""                                 | tee -a $output_file
echo ""                                 | tee -a $output_file
fi

echo "-------- Test Summary --------"   | tee -a $output_file
echo ""                                 | tee -a $output_file
echo "Number of passed test:  $passed"  | tee -a $output_file
echo "Number of failed test:  $failed"  | tee -a $output_file
echo "Number of timeout test: $timeout" | tee -a $output_file

if [ "$failed" -gt 5 ]; then
echo "Summary of failed test: \n$failed_test"
fi

if [ "$timeout" -gt 5 ]; then
echo "Summary of timeout test: \n$timeout_test"
fi

