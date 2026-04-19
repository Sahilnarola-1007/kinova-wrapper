#include "kinova_wrapper/KinovaInterface.hpp"
#include <iostream>
#include <cassert>

int main() {
    using namespace kinova_wrapper;

    std::cout << "=== Test 1: Construction ===\n";
    KinovaInterface kinova;
    assert(!kinova.isConnected());
    std::cout << "PASS: Not connected after construction.\n\n";

    std::cout << "=== Test 2: Connect with valid IP ===\n";
    bool ok = kinova.connect("192.168.1.10");
    assert(ok);
    assert(kinova.isConnected());
    std::cout << "PASS: Connected successfully.\n\n";

    std::cout << "=== Test 3: Connect while already connected (auto-reconnect) ===\n";
    ok = kinova.connect("192.168.1.10", 10000);
    assert(ok);
    assert(kinova.isConnected());
    std::cout << "PASS: Reconnected to new IP.\n\n";

    std::cout << "=== Test 4: Disconnect ===\n";
    kinova.disconnect();
    assert(!kinova.isConnected());
    std::cout << "PASS: Disconnected.\n\n";

    std::cout << "=== Test 5: Disconnect when not connected (safe no-op) ===\n";
    kinova.disconnect();
    assert(!kinova.isConnected());
    std::cout << "PASS: Double disconnect safe.\n\n";

    std::cout << "=== Test 6: Connect with empty IP (should fail) ===\n";
    ok = kinova.connect("");
    assert(!ok);
    assert(!kinova.isConnected());
    std::cout << "PASS: Empty IP rejected.\n\n";

    std::cout << "=== Test 7: Connect with bad credentials (should fail) ===\n";
    ok = kinova.connect("192.168.1.10", 10000, "", "");
    assert(!ok);
    assert(!kinova.isConnected());
    std::cout << "PASS: Bad credentials rejected.\n\n";

    std::cout << "=== Test 8: Destructor cleanup (connect, then let object die) ===\n";
    {
        KinovaInterface temp;
        temp.connect("192.168.1.10");
        // destructor called here — should disconnect cleanly
    }
    std::cout << "PASS: Destructor cleaned up.\n\n";

    std::cout << "=== ALL PART 1 TESTS PASSED ===\n";
    return 0;
}
