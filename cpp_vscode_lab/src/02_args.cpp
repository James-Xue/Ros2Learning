#include <iostream>
#include <string>
#include <vector>

static std::vector<std::string> args_to_vector(int argc, char **argv)
{
    std::vector<std::string> result;
    result.reserve(static_cast<size_t>(argc));

    for (int i = 0; i < argc; ++i)
    {
        result.emplace_back(argv[i]);
    }

    return result;
}

int main(int argc, char **argv)
{
    const auto args = args_to_vector(argc, argv);

    std::cout << "argc=" << argc << "\n";
    for (size_t i = 0; i < args.size(); ++i)
    {
        std::cout << "argv[" << i << "] = " << args[i] << "\n";
    }

    // 练习：
    // 1) 修改 launch.json，给这个程序传入自定义参数
    // 2) 观察 args 的内容
    return 0;
}
