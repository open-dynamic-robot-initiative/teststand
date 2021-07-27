/*
 * Copyright [2017] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

// #include "teststand/teststand.hpp"
// #include "teststand/teststand_ti.hpp"

namespace py = pybind11;
// using namespace teststand;

PYBIND11_MODULE(teststand, m)
{
    // py::class_<Teststand>(m, "Teststand")
    //     .def(py::init<>())
    //     .def("initialize",
    //          py::overload_cast<const std::string&>(&Teststand::initialize))
    //     .def("acquire_sensors", &Teststand::acquire_sensors)
    //     .def("send_target_joint_torque",
    //          &Teststand::send_target_joint_torque,
    //          py::arg("target_joint_torque"))
    //     .def("get_motor_board_errors", &Teststand::get_motor_board_errors)
    //     .def("get_motor_board_enabled", &Teststand::get_motor_board_enabled)
    //     .def("get_motor_enabled", &Teststand::get_motor_enabled)
    //     .def("get_motor_ready", &Teststand::get_motor_ready)
    //     .def("get_joint_positions", &Teststand::get_joint_positions)
    //     .def("get_joint_velocities", &Teststand::get_joint_velocities);

    // py::class_<TeststandTi>(m, "TeststandTi")
    //     .def(py::init<>())
    //     .def("initialize",
    //          py::overload_cast<const std::string&>(&Teststand::initialize))
    //     .def("acquire_sensors", &Teststand::acquire_sensors)
    //     .def("send_target_joint_torque",
    //          &Teststand::send_target_joint_torque,
    //          py::arg("target_joint_torque"))
    //     .def("get_motor_board_errors", &Teststand::get_motor_board_errors)
    //     .def("get_motor_board_enabled", &Teststand::get_motor_board_enabled)
    //     .def("get_motor_enabled", &Teststand::get_motor_enabled)
    //     .def("get_motor_ready", &Teststand::get_motor_ready)
    //     .def("get_joint_positions", &Teststand::get_joint_positions)
    //     .def("get_joint_velocities", &Teststand::get_joint_velocities);
}
