#ifndef TEST_MODULE_H
#define TEST_MODULE_H

void test_servos(struct System_s *sys);
void test_motors(struct System_s *sys);
void test_INS(struct INS_s *ins);
void test_us_sensor();
void test_flight(struct System_s *sys);

#endif // TEST_MODULE_H