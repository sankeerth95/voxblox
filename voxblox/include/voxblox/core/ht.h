
#ifndef VOXBLOX_HT_H_
#define VOXBLOX_HT_H_


#include <iostream>
#include <unordered_map>
#include "/usr/include/eigen3/Eigen/Eigen"
#include "/home/sankeerth/sniper-sim/sniper/include/sim_api.h"

#include "voxblox/core/common.h"

#include "voxblox/core/block_hash.h"

//#define IN_SIM

namespace voxblox {

/**
 * Performs deco hashing on block indexes. Based on recommendations of
 * "Investigating the impact of Suboptimal Hashing Functions" by L. Buckley et
 * al.
 */
struct AnyIndexHash {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// number was arbitrarily chosen with no good justification
  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;

  std::size_t operator()(const AnyIndex& index) const {
    return static_cast<unsigned int>(index.x() + index.y() * sl +
                                     index.z() * sl2);
  }
};




struct LongIndexHash {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;

  std::size_t operator()(const LongIndex& index) const {
    return static_cast<unsigned int>(index.x() + index.y() * sl +
                                     index.z() * sl2);
  }
};

}


template<typename T, typename U>
struct KVPointer{
int tag;
T k;
U v;
};//__attribute__((packed));


enum StoreType{
    LITERAL,				// data stored in the cache is directly the value itself
    PTR_DEREFERENCED		// value stored is indirected: pointer of the object resolved from composite object(like smart pointer, object, etc.).
};


class MyUnorderedMap{


	

};





template<typename T, typename U>
struct HTBase: 
//std::unordered_map<T,U, voxblox::AnyIndexHash> 
std::unordered_map< T, U, voxblox::AnyIndexHash, std::equal_to<T>,
      Eigen::aligned_allocator<std::pair<const T, U> > >
{

	typedef std::unordered_map< T, U, voxblox::AnyIndexHash, std::equal_to<T>,
      Eigen::aligned_allocator<std::pair<const T, U> > > um;

	static short tag;

	HTBase(){
		tag++;
	}


	size_t erase(const T& k){
		#ifdef IN_SIM
		#endif // IN_SIM
		return um::erase(k);
	}

	void clear(){
		#ifdef IN_SIM
		// TODO: add this in?
		#endif // IN_SIM
		um::clear();
	}



  // auto begin() -> decltype(m.begin()){
  //     return m.begin();
  // }
  // auto end() -> decltype(m.end()){
  //     return m.end();
  // }
};

template<typename T, typename U>
short HTBase<T,U>::tag = 0;

template<typename T, typename U, StoreType I>
struct HT;

template<typename T, typename U>
struct HT<T,U,StoreType::LITERAL> : HTBase<T,U> {

    // Send Instruction.
//    uint64_t __attribute__((optimize("O0")))
//    sendInstruction( KVPointer<T,U>& t, const int cmd) const{
//   	return SimUser(cmd, (uint64_t)&t);
//    }


	__attribute__((noinline)) void insert_custom(const std::pair<T,U> x){
		#ifdef IN_SIM
	// TODO: can we not add additional memory allocation
		KVPointer<T,U> t = {this->tag, x.first, x.second};
		long long _res;
		__asm__ __volatile__ (                    \
		"mov %1, %%" MAGIC_REG_A "\n"             \
		"\tmov %2, %%" MAGIC_REG_B "\n"           \
		"\tmov %3, %%" MAGIC_REG_C "\n"           \
		"\txchg %%bx, %%bx\n"                     \
		:  "=g" (_res)          				  \
		: "g"(SIM_CMD_USER),                      \
			"g"((uint64_t)0),                     \
			"g"((uint64_t)&t)               	  \
		: "%" MAGIC_REG_B, "%" MAGIC_REG_C );  

		#endif // IN_SIM
		this->insert(x);
	}



	__attribute__((noinline)) void insert_custom(const T& k, const U& v){
		#ifdef IN_SIM
	// TODO: can we not add additional memory allocation
		KVPointer<T,U> t = {this->tag, k, v};
		long long _res;
		__asm__ __volatile__ (                    \
		"mov %1, %%" MAGIC_REG_A "\n"             \
		"\tmov %2, %%" MAGIC_REG_B "\n"           \
		"\tmov %3, %%" MAGIC_REG_C "\n"           \
		"\txchg %%bx, %%bx\n"                     \
		:  "=g" (_res)          				  \
		: "g"(SIM_CMD_USER),                      \
			"g"((uint64_t)0),                     \
			"g"((uint64_t)&t)               	  \
		: "%" MAGIC_REG_B, "%" MAGIC_REG_C );  

		#endif // IN_SIM
		this->insert(std::make_pair(k,v));
	}

	__attribute__((noinline)) U at_custom(const T& k, bool& found) const {

		#ifdef IN_SIM
		KVPointer<T,U> t = {this->tag, k};
		long long _res;
		__asm__ __volatile__ (                    \
		"mov %1, %%" MAGIC_REG_A "\n"             \
		"\tmov %2, %%" MAGIC_REG_B "\n"           \
		"\tmov %3, %%" MAGIC_REG_C "\n"           \
		"\txchg %%bx, %%bx\n"                     \
		: "=g" (_res)           /* output    */   \
		: "g"(SIM_CMD_USER),                              \
			"g"((uint64_t)1),                             \
			"g"((uint64_t)&t)            /* input     */   \
		: "%" MAGIC_REG_B, "%" MAGIC_REG_C ); /* clobbered */ 
		if(t.v != 0) {
			found = true;
			return t.v;
		}
		#endif // IN_SIM
		
		auto tmp = this->find(k);
		if(tmp != this->end()) {
			found = true;
			return tmp->second;
    } else {
			found = false;
			return U();
		}
	}
};
template<typename T, typename U>
struct HT<T,U,StoreType::PTR_DEREFERENCED> : HTBase<T,U> {

    // Send Instruction.
//   uint64_t __attribute__((optimize("O0")))
//    sendInstruction( KVPointer<T,size_t>& t, const int cmd) const{
//    	return SimUser(cmd, (uint64_t)&t);
//    }
// 
    __attribute__((noinline)) void insert_custom(const T& k, const U& v){
		this->insert(std::make_pair(k,v));
		#ifdef IN_SIM
	// TODO: can we not add additional memory allocation
		KVPointer<T,size_t> t = {this->tag, k, (uint64_t)&v};
		long long _res;
		__asm__ __volatile__ (                    \
		"mov %1, %%" MAGIC_REG_A "\n"             \
		"\tmov %2, %%" MAGIC_REG_B "\n"           \
		"\tmov %3, %%" MAGIC_REG_C "\n"           \
		"\txchg %%bx, %%bx\n"                     \
		:  "=g" (_res)          				  \
		: "g"(SIM_CMD_USER),                      \
			"g"((uint64_t)0),                     \
			"g"((uint64_t)&t)               	  \
		: "%" MAGIC_REG_B, "%" MAGIC_REG_C );  
		#endif // IN_SIM
	}

	__attribute__((noinline)) U* at_custom(const T& k, bool& found) const {
		#ifdef IN_SIM
		KVPointer<T,size_t> t = {this->tag, k, (uint64_t)0};
		unsigned long _res;
		__asm__ __volatile__ (                    \
		"mov %1, %%" MAGIC_REG_A "\n"             \
		"\tmov %2, %%" MAGIC_REG_B "\n"           \
		"\tmov %3, %%" MAGIC_REG_C "\n"           \
		"\txchg %%bx, %%bx\n"                     \
		: "=g" (_res)           /* output    */   \
		: "g"(SIM_CMD_USER),                              \
			"g"((uint64_t)1),                             \
			"g"((uint64_t)&t)            /* input     */   \
		: "%" MAGIC_REG_B, "%" MAGIC_REG_C ); /* clobbered */ 

		if(t.v != 0) {
			found = true;
			return *((U*)t.v);
		}
		#endif // IN_SIM
		
		auto tmp = this->find(k);
		if(tmp != this->end()) {
			found = true;
			return &(*(tmp->second));
			//return (this->at(k));
		}
		else {
			found = false;
			//garbage ptr, stack allocated
			return nullptr;
		}
	}
};


#endif VOXBLOX_HT_H_
