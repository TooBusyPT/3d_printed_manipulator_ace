#include <math.h>
#include <vector>
#include <algorithm>
#include <numeric>

std::vector<float> moving_average_filter(const std::vector<float>& input, int window_size);
std::vector<int> find_local_minima(const std::vector<float>& data);
