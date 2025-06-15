/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   gtest_main.cpp
 * @author Jun Li (junli@hit.edu.cn)
 * @date   April 28, 2025
 */

#include <string>
#include <iostream>
#include <gtest/gtest.h>

int main(int argc, char** argv) 
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}