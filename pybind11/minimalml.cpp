#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <string>

namespace py = pybind11;

// Passing in an array of doubles
// Assume two dimentions always
void show(py::array_t<double> xs) {

    py::buffer_info info = xs.request();
    auto ptr = static_cast<double *>(info.ptr);

    printf("ndim = %zu\n", info.ndim);
    printf("itemsize = %zu\n", info.itemsize);
    printf("shape:\n");
    for(int i = 0; i < info.shape.size(); i++)
        printf("%zu\n", info.shape[i]);
    printf("strides:\n");
    for(int i = 0; i < info.strides.size(); i++)
        printf("%zu\n", info.strides[i]);

//    for(size_t i = 0; i < (info.shape[0] * info.shape[1]); ++i)
//        py::print(ptr[i]);

    for(size_t row = 0; row < info.shape[0]; row++) {
        for(size_t col = 0; col < info.shape[1]; col++) {

            float val = *((ptr+(row * info.shape[1]))+col);
            py::print(val, py::arg("end")=", ");

        }
        py::print("");
    }
}


PYBIND11_PLUGIN(minimalml) {

    py::module m("minimalml", "minimal ml library");

    // Numpy Array operations
    m.def("show", &show);
    return m.ptr();
}
