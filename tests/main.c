#include "greatest.h"

extern SUITE(tune_suite);

GREATEST_MAIN_DEFS();

int main(int argc, char **argv)
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(tune_suite);
    GREATEST_MAIN_END();
}
