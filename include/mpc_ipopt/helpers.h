#ifndef MPC_IPOPT_HELPERS_H
#define MPC_IPOPT_HELPERS_H
// TODO: Move to .cpp file?

// Used like this:
// for (auto i : Range(3, 6)) { // Do something }
// This will loop with i = 3, 4, 5
class Range {
    size_t cur;
    const size_t last;
public:
    Range(size_t start, size_t end) : cur(start), last(end) {}

    // Iterable functions
    const Range &begin() const { return *this; }

    const Range &end() const { return *this; }

    // Iterator functions
    bool operator!=(const Range &r) const { return cur < r.last; }

    void operator++() { ++cur; }

    size_t operator*() const { return cur; }


    size_t operator[](size_t index) const {
        assert(index < length());
        return cur + index;
    }

    size_t length() const { return last - cur; }

    friend Range operator+(const Range &a, const Range &b) {
        assert(a.last == b.cur);
        return {a.cur, b.last};
    }
};

template<typename T1, typename T2>
class zip2 {
    std::pair<T1, T2> containers;
public:
    zip2(T1 t1, T2 t2) : containers(std::make_pair(t1, t2)) {}

    auto begin() { return *this; /*std::make_tuple(std::begin(containers[0]), std::begin(containers[1]));*/ }

    auto end() { return *this; /*std::make_tuple(std::end(containers[0]), std::end(containers[1]));*/ }

    bool operator!=(const zip2 &r) const { return containers.first != r.containers.first; }

    void operator++() {
        ++containers.first;
        ++containers.second;
    }

    auto operator*() const { return std::make_pair(*containers.first, *containers.second); }
};

template<typename T1, typename T2, typename T3>
class zip3 {
    std::tuple<T1, T2, T3> containers;
public:
    zip3(T1 t1, T2 t2, T3 t3) : containers{t1, t2, t3} {}

    auto begin() { return *this; }

    auto end() { return *this; }

    bool operator!=(const zip3 &r) const { return std::get<0>(containers) != std::get<0>(r.containers); }

    void operator++() {
        ++std::get<0>(containers);
        ++std::get<1>(containers);
        ++std::get<2>(containers);
    }

    auto operator*() const {
        return std::make_tuple(*std::get<0>(containers), *std::get<1>(containers), *std::get<2>(containers));
    }
};



// DONT BE STUID

//#include <utility>
//
//template<typename Tuple, size_t... Indices>
//auto dereference_tuple_helper(Tuple &tuple, std::index_sequence<Indices...>) {
//    return std::make_tuple(*(std::get<Indices>(tuple))...);
//}
//
//        auto begin() { return std::make_tuple(std::apply([](auto x) { return std::begin(x); }, containers)); }
//
//        auto end() { return std::make_tuple(std::apply([](auto x) { return std::end(x); }, containers)); }
//
//    };


#endif //MPC_IPOPT_HELPERS_H
