#pragma once

#include <vector>
#include <algorithm>
#include <iostream>
#include <set>



struct V
{
	int8_t x;
	int8_t y;

	V(int8_t , int8_t );
	V();
	
	bool operator <(const V & ) const;
	
	bool operator ==(const V &) const;
	
	bool operator >(const V &) const;
};

struct PairI
{
	int16_t x;
	int16_t y;
	PairI(int16_t, int16_t);
	
};

double dis(V, V);

std::vector<PairI> aStar(V, V, std::vector<std::vector<int16_t>> &);