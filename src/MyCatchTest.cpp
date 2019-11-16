#include <iostream>
#include <vector>

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

using namespace std;
TEST_CASE("vectors can be sized and resized")
{
    vector<int> v(5);
    
    REQUIRE(v.size() == 5);
    REQUIRE(v.capacity() >= 5);

    SECTION("resizing bigger changes size and capacity")
    {
        v.resize(10);
        REQUIRE(v.size() == 10);
        REQUIRE(v.capacity() >= 10);
    }
}