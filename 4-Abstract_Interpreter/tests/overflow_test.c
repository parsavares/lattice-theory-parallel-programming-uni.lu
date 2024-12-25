int z;

void main() {
    z = 2147483647 + 2;  // This should trigger an addition overflow warning
    assert(z == 0);
}