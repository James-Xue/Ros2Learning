#include <iostream>
#include <stdexcept>
#include <string>

class FileHandle
{
  public:
    explicit FileHandle(std::string path)
            : path_(std::move(path))
            , is_open_(true)
    {
        std::cout << "[open ] " << path_ << "\n";
    }

    ~FileHandle()
    {
        if (is_open_)
        {
            std::cout << "[close] " << path_ << "\n";
        }
    }

    FileHandle(const FileHandle &) = delete;
    FileHandle &operator=(const FileHandle &) = delete;

    FileHandle(FileHandle &&other) noexcept
            : path_(std::move(other.path_))
            , is_open_(other.is_open_)
    {
        other.is_open_ = false;
    }

    void write_line(const std::string &line)
    {
        if (!is_open_)
        {
            throw std::runtime_error("handle closed");
        }
        std::cout << "[write] " << line << "\n";
    }

  private:
    std::string path_;
    bool is_open_;
};

static void do_work(bool throw_error)
{
    FileHandle fh("/tmp/demo.txt");
    fh.write_line("hello");

    if (throw_error)
    {
        throw std::runtime_error("boom");
    }

    fh.write_line("world");
}

int main()
{
    try
    {
        do_work(false);
        do_work(true);
    }
    catch (const std::exception &e)
    {
        std::cout << "Caught exception: " << e.what() << "\n";
    }

    // 练习：
    // 1) 用断点观察异常抛出点与栈
    // 2) 改造 FileHandle：把 is_open_ 改成更真实的资源（比如 FILE*）
    return 0;
}
