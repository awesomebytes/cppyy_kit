#include "native.hpp"
#include <mcap/reader.hpp>
#include <chrono>
#include <cstring>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <map>
#include <fstream>
namespace typed_mcap {
using Clock = std::chrono::steady_clock;
uint64_t count_speed_above(const uint64_t* timestamps,const double* p,size_t count,double threshold) {
  uint64_t matches=0;
  for(size_t i=1;i<count;++i) {
    if(timestamps[i]<timestamps[i-1]) throw std::runtime_error("decreasing timestamps");
    if(timestamps[i]==timestamps[i-1]) continue; // Duplicate observations have no speed interval.
    long double dx=static_cast<long double>(p[3*i])-p[3*i-3];
    long double dy=static_cast<long double>(p[3*i+1])-p[3*i-2];
    long double dz=static_cast<long double>(p[3*i+2])-p[3*i-1];
    long double dt=static_cast<long double>(timestamps[i]-timestamps[i-1])*1e-9L;
    if(std::hypot(dx,dy,dz)/dt>static_cast<long double>(threshold)) ++matches;
  }
  return matches;
}
double elapsed(Clock::time_point t) {
  return std::chrono::duration<double, std::milli>(Clock::now()-t).count();
}
// Compare every field, dependency and field order. Ignore comments and whitespace.
// Dependency section ordering is immaterial; no extra or missing fields are accepted.
std::map<std::string,std::vector<std::string>> layout(const std::string& text) {
  std::map<std::string,std::vector<std::string>> out;
  std::string section="root", line;
  std::istringstream stream(text);
  while(std::getline(stream,line)) {
    line=line.substr(0,line.find('#'));
    std::istringstream words(line); std::string a,b,c;
    if(!(words>>a) || a[0]=='=') continue;
    if(a=="MSG:") { if(!(words>>section)) throw std::runtime_error("invalid schema section"); }
    else { if(!(words>>b) || (words>>c)) throw std::runtime_error("unsupported schema syntax"); out[section].push_back(a+" "+b); }
  }
  return out;
}
struct Cdr {
  const uint8_t* p; size_t n, i=4; bool little;
  Cdr(const std::byte* data,size_t size):p(reinterpret_cast<const uint8_t*>(data)),n(size) {
    if(n<4 || p[0]!=0 || p[1]>1 || p[2]!=0 || p[3]!=0)
      throw std::runtime_error("only CDR v1 BE/LE with zero options supported");
    little=p[1]==1;
  }
  void need(size_t count) { if(i>n || count>n-i) throw std::runtime_error("truncated CDR payload"); }
  uint64_t integer(size_t bytes) {
    i += (bytes-(i-4)%bytes)%bytes; need(bytes); uint64_t value=0;
    for(size_t k=0;k<bytes;++k) value |= uint64_t(p[i+k]) << (8*(little?k:bytes-k-1));
    i+=bytes; return value;
  }
  double real() { uint64_t bits=integer(8); double v; std::memcpy(&v,&bits,8); if(!std::isfinite(v)) throw std::runtime_error("nonfinite pose"); return v; }
  std::string string() {
    size_t count=integer(4); if(count==0) throw std::runtime_error("zero length CDR string");
    need(count); if(p[i+count-1]!=0 || std::memchr(p+i,0,count-1)) throw std::runtime_error("invalid CDR string");
    std::string result(reinterpret_cast<const char*>(p+i),count-1); i+=count; return result;
  }
  int64_t stamp() { int64_t sec=static_cast<int32_t>(integer(4)); uint64_t ns=integer(4); if(ns>=1000000000) throw std::runtime_error("invalid header nanosec"); return sec*1000000000LL+ns; }
  void finish() { if(i!=n) throw std::runtime_error("unexpected trailing CDR fields"); }
};
Batch extract(const std::string& path,const std::string& topic,uint64_t start,
              uint64_t end,bool image,bool headers_only,const std::string& expected) {
  if(end<start) throw std::runtime_error("invalid time interval");
  auto beginning=Clock::now(); Batch out; if(image) out.offsets.push_back(0);
  // The reader's fallback scan can recover a truncated file. This API requires
  // a complete footer and final magic, so recovery is not mistaken for validity.
  {
    std::ifstream file(path,std::ios::binary|std::ios::ate);
    auto size=file.tellg();
    if(!file || size<37) throw std::runtime_error("MCAP: missing complete footer");
    file.seekg(-37,std::ios::end); unsigned char footer[37];
    file.read(reinterpret_cast<char*>(footer),37);
    const unsigned char magic[8]={0x89,'M','C','A','P','0','\r','\n'};
    if(!file || footer[0]!=2 || footer[1]!=20 ||
       std::memcmp(footer+29,magic,8)!=0)
      throw std::runtime_error("MCAP: invalid footer or final magic");
    for(int k=2;k<9;++k) if(footer[k]!=0) throw std::runtime_error("MCAP: invalid footer length");
  }
  mcap::McapReader reader;
  auto problem=[](const mcap::Status& s){ if(!s.ok()) throw std::runtime_error("MCAP: "+s.message); };
  problem(reader.open(path)); problem(reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan,problem));
  // This build deliberately rejects compressed chunks. No decompression timing is claimed.
  for(const auto& index:reader.chunkIndexes())
    if(!index.compression.empty()) throw std::runtime_error("compressed chunks unsupported by this experiment; convert to uncompressed MCAP");
  bool found=false;
  const auto expected_layout=layout(expected);
  auto check=[&](const mcap::ChannelPtr& channel,const mcap::SchemaPtr& schema){
    if(!schema || schema->name!=(image?"sensor_msgs/msg/Image":"geometry_msgs/msg/PoseStamped") ||
       schema->encoding!="ros2msg" || channel->messageEncoding!="cdr") throw std::runtime_error("unsupported topic schema or encoding");
    std::string definition(reinterpret_cast<const char*>(schema->data.data()),schema->data.size());
    if(layout(definition)!=expected_layout) throw std::runtime_error("schema layout differs from supported ROS 2 definition");
  };
  for(const auto& pair:reader.channels()) if(pair.second->topic==topic) {
    found=true; auto schema=reader.schema(pair.second->schemaId); check(pair.second,schema);
  }
  if(!found) throw std::runtime_error("missing topic: "+topic);
  mcap::ReadMessageOptions options; options.startTime=start; options.endTime=end;
  options.topicFilter=[&](std::string_view t){return t==topic;};
  bool first=true; uint64_t previous=0; std::unordered_set<uint16_t> checked;
  for(const auto& view:reader.readMessages(problem,options)) {
    if(checked.insert(view.channel->id).second) check(view.channel,view.schema);
    if(!first && view.message.logTime<previous) throw std::runtime_error("decreasing topic log timestamp");
    first=false; previous=view.message.logTime;
    auto decoding=Clock::now(); Cdr c(view.message.data,view.message.dataSize);
    int64_t stamp=c.stamp(); std::string frame=c.string();
    if(out.header_ns.empty()) out.frame_id=frame;
    if(frame!=out.frame_id) throw std::runtime_error("mixed frame_id unsupported");
    out.log_ns.push_back(view.message.logTime); out.publish_ns.push_back(view.message.publishTime); out.header_ns.push_back(stamp);
    if(!image) {
      for(int k=0;k<3;++k) out.position.push_back(c.real());
      for(int k=0;k<4;++k) out.quaternion.push_back(c.real());
    } else {
      uint32_t h=c.integer(4),w=c.integer(4); auto encoding=c.string(); auto big=c.integer(1); uint32_t step=c.integer(4),count=c.integer(4);
      if((encoding!="rgb8" && encoding!="bgr8") || big>1 || uint64_t(w)*3>step || uint64_t(h)*step!=count)
        throw std::runtime_error("only valid rgb8/bgr8 Image layout supported");
      if(!out.encoding.empty() && encoding!=out.encoding) throw std::runtime_error("mixed Image encodings unsupported");
      c.need(count); if(!headers_only) out.data.insert(out.data.end(),c.p+c.i,c.p+c.i+count);
      c.i+=count; out.offsets.push_back(out.data.size()); out.width.push_back(w); out.height.push_back(h); out.step.push_back(step); out.is_bigendian.push_back(big); out.encoding=encoding;
    }
    c.finish(); out.decode_ms+=elapsed(decoding);
  }
  out.container_ms=elapsed(beginning)-out.decode_ms; return out;
}
}
