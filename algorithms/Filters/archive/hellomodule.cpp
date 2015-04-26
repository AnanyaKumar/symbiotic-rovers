#include <iostream>
#include <vector> 
#include <list>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/object.hpp>
using namespace std;
using namespace boost::python;
 
struct World
{
    std::string greet() { return msg; }
    std::string msg;
};

vector<double> to_vector(boost::python::list ns) {
  vector<double> v;
  for (int i = 0; i < len(ns); ++i)
  {
    int x;
    x = boost::python::extract<double>(ns[i]);
    v.push_back(x);
  }
  return v;
}

void lol (boost::python::list ns) {
  vector<double> v = to_vector(ns);
  for (int i = 0; i < v.size(); i++) {
    cout << v[i] << endl;
  }
}


BOOST_PYTHON_MODULE(hello)
{
  def("to_vector", &lol);
}

