#include "greatest.h"

extern SUITE(tune_suite);
extern SUITE(yolo_suite);

GREATEST_MAIN_DEFS();

int main(int argc, char **argv)
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(tune_suite);
    RUN_SUITE(yolo_suite);
    GREATEST_MAIN_END();
}
