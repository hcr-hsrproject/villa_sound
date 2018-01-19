/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2010, Kyoto University and Honda Motor Co.,Ltd. All rights reserved.
 *
 * HARK was developed by researchers in Okuno Laboratory at the Kyoto University and 
 * Honda Research Institute Japan Co.,Ltd.
 *
 * Redistribution and use in source and binary forms, with or without modification, are 
 * permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list 
 *    of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice, this 
 *    list of conditions and the following disclaimer in the documentation and/or other 
 *    materials provided with the distribution.
 *  * Neither the name of Kyoto University and Honda Motor Co.,Ltd., Inc. nor the names 
 *    of its contributors may be used to endorse or promote products derived from this 
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCI-
 * DENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSI-
 * NESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTR-
 * ACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _RECOGSOURCE_H_
#define _RECOGSOURCE_H_

#include "Object.h"
#include "ObjectParser.h"
#include "misc.h"
#include <cmath>

using namespace FD;
using namespace std;

class RecogSource : public Object{
  
 public:

  typedef struct tag_ASR_SRC {
    string word;    
    int classid; 
    string phone;
    float cm;
  } ASR_SRC;

  int id;
  float azimuth;
  float elevation;
  int sec;
  int usec;
  int frames;
  int msec;
  int status;
  vector<ASR_SRC> data;  

  RecogSource(){
  }

  RecogSource(const RecogSource& input) {
    id        = input.id;
    azimuth   = input.azimuth;
    elevation = input.elevation;
    sec       = input.sec;
    usec      = input.usec;
    frames    = input.frames;
    msec      = input.msec;
    status    = input.status;
    data.resize(input.data.size());
    for(int cnt = 0; cnt < input.data.size(); cnt++){
      data[cnt].word = input.data[cnt].word;
      data[cnt].classid = input.data[cnt].classid;
      data[cnt].phone = input.data[cnt].phone;
      data[cnt].cm = input.data[cnt].cm;      
    }
  }
      
  ~RecogSource(){
  }
    
  virtual ObjectRef clone(){
    return ObjectRef(new RecogSource(*this));
  }
                
  /**Serialize (binary) the object to a stream*/
  virtual void serialize(std::ostream &out) const {
  }
  
  /**Unserialize (binary) the object from a stream*/
  virtual void unserialize(std::istream &in){
  }
                
  /**Generic print function*/
  virtual void printOn(std::ostream &out=std::cout) const {
    out << "<Source " << std::endl;
    out << "<id " << id << " >" << std::endl;
    out << "<azimuth " << azimuth << " >" << std::endl;
    out << "<elevation " << elevation << " >" << std::endl;
    out << "<status " << status << " >" << std::endl;
    out << ">" << std::endl;
  }
     
  /**Generic read function*/
  virtual void readFrom(std::istream &in=std::cin){
  }      
      
};

#endif

