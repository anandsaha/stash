#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <string>

namespace py = pybind11;

int add(int i, int j) {
    return i + j;
}


class Pet {
    public:
        Pet(const std::string& name) : _name(name) {}

        void setName(const std::string& name) {
            _name = name;
        }

        const std::string& getName() const {
            return _name;
        }

    private:
        std::string _name;

};


// Passing in an array of doubles
void twice(py::array_t<double> xs) {
    py::buffer_info info = xs.request();
    auto ptr = static_cast<double *>(info.ptr);

    printf("ndim = %d\n", info.ndim);
    printf("itemsize = %d\n", info.itemsize);
    printf("shape:\n");
    for(int i = 0; i < info.shape.size(); i++)
        printf("%d\n", info.shape[i]);
    printf("strides:\n");
    for(int i = 0; i < info.strides.size(); i++)
        printf("%d\n", info.strides[i]);



    int n = 1;
    for (auto r: info.shape) {
        n *= r;
    }

    for (int i = 0; i <n; i++) {
        *ptr++ *= 2;
    }
}

// Passing in a generic array
double sum(py::array xs) {
    py::buffer_info info = xs.request();
    auto ptr = static_cast<double *>(info.ptr);

    int n = 1;
    for (auto r: info.shape) {
        n *= r;
    }

    double s = 0.0;
    for (int i = 0; i <n; i++) {
        s += *ptr++;
    }

    return s;
}


PYBIND11_PLUGIN(example) {

    py::module m("example", "pybind11 example plugin");

    // Method
    m.def("add", &add, "A function which adds two numbers", py::arg("i"), py::arg("j"));

    // Attributes
    m.attr("the_answer") = 42;
    py::object world = py::cast("World");
    m.attr("what") = world;

    // Class
    py::class_<Pet>(m, "Pet")
        .def(py::init<const std::string&>())
        .def("setName", &Pet::setName)
        .def("getName", &Pet::getName)
        .def("__repr__", [](const Pet& p) { return "<example.Pet named '" + p.getName() + "'>"; })
        .def_property("name", &Pet::getName, &Pet::setName)
        ;

    // Numpy Array operations
    m.def("sum", &sum);
    m.def("twice", &twice);
    return m.ptr();
}
