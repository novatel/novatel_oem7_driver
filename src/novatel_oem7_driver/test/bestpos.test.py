################################################################################
# Copyright (c) 2021 NovAtel Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
#################################################################################


from novatel_oem7_driver import testutil
import launch_testing

#@pytest.mark.launc_test
def generate_test_description():
    return testutil.generate_test_description('bestpos', 
                                              ['/novatel/oem7/bestpos',
                                               '/novatel/oem7/bestvel', 
                                               '/novatel/oem7/bestutm', 
                                               '/novatel/oem7/inspva',
                                               '/novatel/oem7/odom', 
                                               '/novatel/oem7/gps', 
                                               '/novatel/oem7/fix']
                                              )

    
class ConcurrentTestWorkaround(testutil.ConcurrentTestWorkaround):
    pass

@launch_testing.post_shutdown_test()
class BestposBagEquivalencyTest(testutil.BagEquivalencyTest):
    pass

