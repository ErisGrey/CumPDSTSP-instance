#include <iostream>
#include "config.h"
#include "C:/Users/admin/Downloads/instance.h"
int main(int argc, char* argv[])
{
    Config config(argc, argv);
    
    Instance instance(config.input,config.param_input);

    
    return 0;
}