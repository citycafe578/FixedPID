// Host entry point for test/test.cpp: runs setup() once and reports the result
// through the process exit code (0 = all tests passed).
void setup();
extern int totalTests;
extern int passedTests;

int main() {
    setup();
    return (totalTests > 0 && passedTests == totalTests) ? 0 : 1;
}
