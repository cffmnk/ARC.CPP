#pragma once

#include <vector>
#include <algorithm>
#include <iostream>
#include <set>

#define pii std::pair<int, int>

double dis(pii, pii);

std::vector<std::pair<int, int>> aStar(pii, pii, std::vector<std::vector<int16_t>> &);