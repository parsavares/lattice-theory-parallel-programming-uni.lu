int a;
int b;

void main() {
    a = 10;
    // Deliberate division by zero
    b = a / 0;
    // Optionally, an assertion (not strictly needed for your check)
    assert(b == 5);
}
