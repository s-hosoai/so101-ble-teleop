#include "peer_store.h"

#include <Preferences.h>

namespace peerstore {

static constexpr const char* kNs = "so101";

bool load(NimBLEAddress& out) {
  Preferences p;
  p.begin(kNs, true);
  const String s = p.getString("peer", "");
  const uint8_t type = p.getUChar("ptype", BLE_ADDR_PUBLIC);
  p.end();
  if (s.length() != 17) return false;  // "aa:bb:cc:dd:ee:ff"
  out = NimBLEAddress(std::string(s.c_str()), type);
  return true;
}

void save(const NimBLEAddress& addr) {
  Preferences p;
  p.begin(kNs, false);
  p.putString("peer", addr.toString().c_str());
  p.putUChar("ptype", addr.getType());
  p.end();
}

void clear() {
  Preferences p;
  p.begin(kNs, false);
  p.remove("peer");
  p.remove("ptype");
  p.end();
}

}  // namespace peerstore
