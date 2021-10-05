#include <iostream>
#include <unordered_map>
#include "/usr/include/eigen3/Eigen/Eigen"
#include "/home/sankeerth/sniper-sim/sniper/include/sim_api.h"

#include "voxblox/core/common.h"

//#define IN_SIM

typedef Eigen::Vector3i ChunkID;

struct ChunkHasher
{
    // Three large primes are used for spatial hashing.
    static constexpr size_t p1 = 73856093;
    static constexpr size_t p2 = 19349663;
    static constexpr size_t p3 = 83492791;

    std::size_t operator()(const ChunkID& key) const
    {
        return ( key(0) * p1 ^ key(1) * p2 ^ key(2) * p3);
    }
};


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
struct HTBase{

	std::unordered_map<T, U, ChunkHasher> m;
	static short tag;

	HTBase(){
		tag++;
	}

    size_t size() const{
		return m.size();
	}

	void erase_custom(const T& k){
		#ifdef IN_SIM
		#endif // IN_SIM
		m.erase(k);
	}

	void clear_custom(){
		#ifdef IN_SIM
		// TODO: add this in?
		#endif // IN_SIM
		m.clear();
	}

	// added by Jenny 
	U operator[] (const T& k){
        return m[k];
    }

	// added by Jenny 
	bool empty() const {
        return m.empty();
    }

    auto begin() -> decltype(m.begin()){
        return m.begin();
    }
    auto end() -> decltype(m.end()){
        return m.end();
    }
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
		this->m.insert(std::make_pair(k,v));
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
		
		auto tmp = this->m.find(k);
		if(tmp != this->m.end()) {
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

    __attribute__((noinline)) void insert_custom(const T& k, const U& v){
		this->m.insert(std::make_pair(k,v));
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
			return ((U*)t.v);
		}
		#endif // IN_SIM
		
		auto tmp = this->find(k);
		if(tmp != this->end()) {
			found = true;
			return &(tmp->second);
//			return (this->at(k));
		}
		else {
			found = false;
			//garbage ptr, stack allocated
			return nullptr;
		}
	}
};


