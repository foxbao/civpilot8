/******************************************************************************
 * Copyright 2022 The CIV Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once
#include <map>
#include <string>
#include "vtk_source/source_base.hpp"
namespace civ {
namespace civview {
class VehicleSource: public SourceBase {
 public:
 using SourceBase::SourceBase;  // 继承构造函数
  VehicleSource();

  // VehicleSource(SourceBaseMode const& mode, std::string const& path)
  //     : SourceBase(mode, path) {
  // }

  bool ReadFile(std::string file_path);

protected:
  void GenerateActorImpl(std::map<size_t, sp_cMsg2>& msgs) override;
  sp_cMsg2 AdapteProto(sp_cMsg msg) override;

 private:
  std::string file_path_;
};
}  // namespace civview
}  // namespace civ
