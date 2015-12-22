#ifndef _REVERTABLE_H
#define _REVERTABLE_H

namespace mySLAM
{
  template<typename T>
  class Revertable
  {
  public:
    Revertable() :
      value()
    {}
    Revertable(const T& value) :
      value(value)
    {}

    inline const T& operator()() const
    {
      return value;
    }

    T& update()
    {
      old = value;
    }
    void revert()
    {
      value = old;
    }
  private:
    T old, value;
  };

} // end namespace mySLAM

#endif
