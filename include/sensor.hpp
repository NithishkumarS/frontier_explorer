/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2018, Nantha Kumar Sunder, Nithish Sanjeev Kumar 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.

 * * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**@file sensor.hpp
 *
 * @brief To declare a class which implements radiation sensor module
 *
 * @driver Nantha Kumar Sunder
 * @navigator Nithish Sanjeev Kumar
 * @copyright 2018 , Nantha Kumar Sunder, Nithish Sanjeev Kumar All rights reserved

 */
#pragma once
#include <vector>
#include "map.hpp"

/**
 *  @brief Class to implement the radiation sensor
 *
 *  @param radiationMap that store the intensity of the radiation
 *
 *  @return None
 */
class sensor {
 private:
  // radiationMap is the vector that contains the radiation intensity
  std::vector<std::vector<float>> radiationMap;

 public:
  /**
   *  @brief class constructor
   *
   *  @param Map object to get number of rows and column in the map
   *
   *  @return None
   */
  explicit sensor(map&);
  /**
   *  @brief Class to save the radiation data as ppm file
   *
   *  @return None
   */
  void saveRadiationMap();
  /**
   *  @brief Class to implement the BFS
   *
   *  @return radiationMap in 2-D vector
   */
  std::vector<std::vector<float>> getRadiationMap();
  ~sensor();
};

