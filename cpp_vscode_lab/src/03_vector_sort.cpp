#include <algorithm>
#include <iostream>
#include <vector>

static void print(const std::vector<int> &v)
{
    for (size_t i = 0; i < v.size(); ++i)
    {
        std::cout << v[i] << (i + 1 == v.size() ? "\n" : " ");
    }
}

int main()
{
    std::vector<int> numbers{5, 1, 4, 2, 8, 0, 3};

    std::cout << "Before: ";
    print(numbers);

    std::sort(numbers.begin(), numbers.end(), [](int a, int b) { return a < b; });

    std::cout << "After : ";
    print(numbers);

    // 练习：
    // 1) 改成降序排序
    // 2) 给 numbers push_back 一些值（含重复），再去重（unique + erase）
    return 0;
}
