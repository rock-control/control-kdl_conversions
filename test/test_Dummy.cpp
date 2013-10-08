#include <boost/test/unit_test.hpp>
#include <kdl_conversions/Dummy.hpp>

using namespace kdl_conversions;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    kdl_conversions::DummyClass dummy;
    dummy.welcome();
}
