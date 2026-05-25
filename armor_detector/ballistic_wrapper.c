// 在 armor_detector/armor_detector/ 目录下创建 ballistic_wrapper.c
#include <Python.h>
#include "SolveTrajectory.h"

static PyObject* wrapper_autoSolveTrajectory(PyObject* self, PyObject* args) {
    float pitch, yaw, aim_x, aim_y, aim_z;
    autoSolveTrajectory(&pitch, &yaw, &aim_x, &aim_y, &aim_z);
    return Py_BuildValue("(fffff)", pitch, yaw, aim_x, aim_y, aim_z);
}

static PyMethodDef BallisticMethods[] = {
    {"auto_solve", wrapper_autoSolveTrajectory, METH_VARARGS, "弹道解算"},
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef ballisticmodule = {
    PyModuleDef_HEAD_INIT, "ballistic", NULL, -1, BallisticMethods
};

PyMODINIT_FUNC PyInit_ballistic(void) {
    return PyModule_Create(&ballisticmodule);
}