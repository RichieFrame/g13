/*
 * This file contains code for managing keys and profiles
 */
#include "g13.hpp"

namespace G13 {

// *************************************************************************

void G13_Device::parse_joystick(unsigned char *buf) {
  m_stick.ParseJoystick(buf);
}

G13_Stick::G13_Stick(G13_Device &keypad)
    : _keypad(keypad), m_bounds(0, 0, 255, 255), m_center_pos(127, 127),
      m_north_pos(127, 0) {
  m_stick_mode = STICK_KEYS;

  auto add_zone = [this, &keypad](const std::string &name, double x1, double y1,
                                  double x2, double y2) {
    m_zones.emplace_back(
        *this, "STICK_" + name, G13_ZoneBounds(x1, y1, x2, y2),
        G13_ActionPtr(new G13_Action_Keys(keypad, "KEY_" + name)));
  };

  add_zone("UP", 0.0, 0.0, 1.0, 0.3);
  add_zone("DOWN", 0.0, 0.7, 1.0, 1.0);
  add_zone("LEFT", 0.0, 0.0, 0.3, 1.0);
  add_zone("RIGHT", 0.7, 0.0, 1.0, 1.0);
  //add_zone("PAGEUP", 0.0, 0.0, 1.0, 0.1);
  //add_zone("PAGEDOWN", 0.0, 0.9, 1.0, 1.0);
}

G13_StickZone *G13_Stick::zone(const std::string &name, bool create) {
  for (auto &zone : m_zones) {
    if (zone.name() == name) {
      return &zone;
    }
  }
  if (create) {
    m_zones.push_back(
        G13_StickZone(*this, name, G13_ZoneBounds(0.0, 0.0, 0.0, 0.0)));
    return zone(name);
  }
  return nullptr;
}

void G13_Stick::set_mode(stick_mode_t m) {
  if (m == m_stick_mode)
    return;
  if (m_stick_mode == STICK_CALCENTER || m_stick_mode == STICK_CALBOUNDS ||
      m_stick_mode == STICK_CALNORTH) {
    RecalcCalibrated();
  }
  m_stick_mode = m;
  switch (m_stick_mode) {
  case STICK_CALBOUNDS:
    m_bounds.tl = G13_StickCoord(255, 255);
      m_bounds.br = G13_StickCoord(0, 0);
    break;
  case STICK_ABSOLUTE:
    break;
  case STICK_KEYS:
    break;
  case STICK_CALCENTER:
    break;
  case STICK_CALNORTH:
    break;
  }
}

void G13_Stick::RecalcCalibrated() {}

void G13_Stick::RemoveZone(const G13_StickZone &zone) {
  const G13_StickZone &target(zone);
  m_zones.erase(std::remove(m_zones.begin(), m_zones.end(), target), m_zones.end());
}
void G13_Stick::dump(std::ostream &out) const {
  for (auto &zone : m_zones) {
    zone.dump(out);
    out << std::endl;
  }
}

void G13_StickZone::dump(std::ostream &out) const {
  out << "   " << std::setw(20) << name() << "   " << _bounds << "  ";
  if (action()) {
    action()->dump(out);
  } else {
    out << " (no action)";
  }
}

void G13_StickZone::test(const G13_ZoneCoord &loc) {
  if (_action && _active != _bounds.contains(loc))
    _action->act(_active = !_active);
}

void checkaxis(const unsigned char x, const unsigned char y, G13_StickZone &m_zone1, G13_StickZone &m_zone2) {
  if (y < 127 - 50 || y > 127 + 50) {
    unsigned char x2 = x>>1;
    if (x2 + 64 > y) {
      if (191 - x2 > y) {
        m_zone1.latch(true);
        m_zone2.latch(false);
      } else {
        m_zone1.latch(false);
        m_zone2.latch(false);
      }
    } else if (191 - x2 < y) {
      m_zone1.latch(false);
      m_zone2.latch(true);
    } else {
      m_zone1.latch(false);
      m_zone2.latch(false);
    }
  } else {
    m_zone1.latch(false);
    m_zone2.latch(false);
  }
}

void G13_StickZone::latch(const bool state) {
  if (_action && _active != state)
    _action->act(_active = !_active);
}

G13_StickZone::G13_StickZone(G13_Stick &stick, const std::string &name,
                             const G13_ZoneBounds &b,
                             const G13_ActionPtr &action)
    : G13_Actionable<G13_Stick>(stick, name), _bounds(b), _active(false) {
  set_action(action); // Call to virtual from ctor!
}

void G13_Stick::ParseJoystick(const unsigned char *buf) {
  if (m_stick_mode == STICK_KEYS) {
    checkaxis(buf[1],buf[2],m_zones[0],m_zones[1]);
    checkaxis(buf[2],buf[1],m_zones[2],m_zones[3]);
    return;
  }
  //m_current_pos.x = buf[1];
  //m_current_pos.y = buf[2];

  // update targets if we're in calibration mode
  /*switch (m_stick_mode) {
  case STICK_ABSOLUTE:
    break;
  case STICK_KEYS:
    break;

  case STICK_CALCENTER:
    m_center_pos = m_current_pos;
    return;

  case STICK_CALNORTH:
    m_north_pos = m_current_pos;
    return;

  case STICK_CALBOUNDS:
    m_bounds.expand(m_current_pos);
    return;
  }
  if (m_stick_mode == STICK_ABSOLUTE) {
    _keypad.SendEvent(EV_ABS, ABS_X, m_current_pos.x);
    _keypad.SendEvent(EV_ABS, ABS_Y, m_current_pos.y);

  } else if (m_stick_mode == STICK_KEYS) {
    if (m_current_pos.y < 127 - 50 || m_current_pos.y > 127 + 50) {
      unsigned char x2 = m_current_pos.x>>1;
      if (x2 + 64 > m_current_pos.y) {
	if (191 - x2 > m_current_pos.y) {
          //G13_OUT("up DOWN");
	  m_zones[0].latch(true);
	  m_zones[1].latch(false);
	} else {
          //G13_OUT("up down");
	  m_zones[0].latch(false);
	  m_zones[1].latch(false);
	}
      } else if (191 - x2 < m_current_pos.y) {
        //G13_OUT("UP down");
        m_zones[0].latch(false);
        m_zones[1].latch(true);
      } else {
        //G13_OUT("up down");
        m_zones[0].latch(false);
        m_zones[1].latch(false);
      }
    } else {
      //G13_OUT("up down");
      m_zones[0].latch(false);
      m_zones[1].latch(false);
    }
    if (m_current_pos.x < 127 - 50 || m_current_pos.x > 127 + 50) {
      unsigned char y2 = m_current_pos.y>>1;
      if (y2 + 64 > m_current_pos.x) {
        if (191 - y2 > m_current_pos.x) {
          //G13_OUT("RIGHT left");
          m_zones[2].latch(true);
          m_zones[3].latch(false);
	} else {
          //G13_OUT("right left");
          m_zones[2].latch(false);
          m_zones[3].latch(false);
	}
      } else if (191 - y2 < m_current_pos.x) {
        //G13_OUT("right LEFT");
        m_zones[2].latch(false);
        m_zones[3].latch(true);
      } else {
        //G13_OUT("right left");
        m_zones[2].latch(false);
        m_zones[3].latch(false);
      }
    } else {
      //G13_OUT("right left");
      m_zones[2].latch(false);
      m_zones[3].latch(false);
    }
    return;

    // BOOST_FOREACH (G13_StickZone& zone, m_zones) { zone.test(jpos); }
    for (auto &zone : m_zones) {
      zone.test(G13_ZoneCoord(m_current_pos.x, m_current_pos.y));
    }
    return;

  } else {*/
    /*    send_event(g13->uinput_file, EV_REL, REL_X, stick_x/16 - 8);
     SendEvent(g13->uinput_file, EV_REL, REL_Y, stick_y/16 - 8);*/
  //}
}

} // namespace G13
