#ifndef _RGBD_PAIR_H_
#define _RGBD_PAIR_H_

// including RgbdPair and FileReader classes.
#include <iostream>
#include <fstream>
#include <vector>

namespace mySLAM
{
  class RgbdPair
  {
  public:
    RgbdPair() {};
    ~RgbdPair() {}; // adding virtual?

    std::string RgbTimestamp, RgbFile;
    std::string DepTimestamp, DepFile;

    friend std::ostream& operator <<(std::ostream& out, const RgbdPair& pair);
    friend std::istream& operator >>(std::istream& in , RgbdPair& pair);
  };

  std::ostream& operator <<(std::ostream& out, const RgbdPair& pair)
  {
    out
      <<pair.RgbFile << " "
      <<pair.DepFile << std::endl;

    return out;
  }

  std::istream& operator >>(std::istream& in, RgbdPair& pair)
  {
    // what we need is just RgbFile and DepFile
    double time;
    in >> time;
    in >> pair.RgbFile;
    in >> time;
    in >> pair.DepFile;

    return in;
  }

  class FileReader
  {
  public:
    FileReader(std::string& file): file_(file), file_stream_(file.c_str()){}

    virtual ~FileReader()
    {
      file_stream_.close();
    }

    bool next()
    {
      if(file_stream_.good() && !file_stream_.eof())
      {
        file_stream_ >> pair;

        return true;
      }
      return false;
    }

    void readAllEntries(std::vector<RgbdPair>& pairs)
    {
      while(next())
      {
        pairs.push_back(pair);
      }
      pairs.pop_back(); // To delete the last repeat pair
    }

  private:
    std::string file_;
    std::ifstream file_stream_;
    RgbdPair pair;
  }; // end class FileReader

} // end namespace mySLAM
#endif
