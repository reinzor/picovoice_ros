#include "./util.h"

namespace picovoice_driver
{
KeyValue::KeyValue(const std::string& key, const std::string& value) : key_(key), value_(value)
{
}

std::ostream& operator<<(std::ostream& os, const KeyValue& kv)
{
  os << "{" << kv.key_ << "=" << kv.value_ << "}";
  return os;
}
}  // namespace picovoice_driver
