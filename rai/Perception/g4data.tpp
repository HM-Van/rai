/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

template <class T>
rai::Array<T> G4ID::query(const rai::Array<T> &data, const String &sensor) {
  return data[hsi(sensor)];
}

template <class T>
rai::Array<T> G4ID::query(const rai::Array<T>  &data, const StringA &sensors) {
  rai::Array<T> x;
  for(const String &sensor: sensors) {
    x.append(data[hsi(sensor)]);
  }
  uint nsensors = sensors.N;
  x.reshape(nsensors, x.N/nsensors);
  return x;
}

template<class T>
void G4Rec::set(const char *key, const T &value) {
  Node *i = params.getNode(key);
  if(i)
    i->get<T>() = value;
  else
    params.append(key, new T(value));
}

template<class T>
bool G4Rec::get(const char *key, T &value) {
  return params.get(value, key);
}

template<class T>
T* G4Rec::get(const char *key) {
  return params.find<T>(key);
}

