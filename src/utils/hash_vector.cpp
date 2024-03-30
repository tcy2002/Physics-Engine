template <typename T, typename HashFunc, typename EqualFunc>
bool hash_vector<T, HashFunc, EqualFunc>::push_back(const T& item) {
    uint32_t hash = (uint32_t)hash_func(item);
    auto range = index_table.equal_range(hash);
    for (auto it = range.first; it != range.second; it++) {
        if (equal_func(list[it->second], item)) {
            return false;
        }
    }
    index_table.insert({hash, size()});
    list.push_back(item);
    return true;
}

template <typename T, typename HashFunc, typename EqualFunc>
void hash_vector<T, HashFunc, EqualFunc>::pop_back() {
    if (list.empty()) {
        return;
    }
    uint32_t hash = (uint32_t)hash_func(list.back());
    auto range = index_table.equal_range(hash);
    for (auto it = range.first; it != range.second; it++) {
        if (it->second == size() - 1) {
            index_table.erase(it);
            break;
        }
    }
    list.pop_back();
}

template <typename T, typename HashFunc, typename EqualFunc>
bool hash_vector<T, HashFunc, EqualFunc>::insert(typename std::vector<T>::const_iterator iterator,
                                                             const T &item) {
    if (iterator < list.begin() || iterator > list.end()) {
        return false;
    } else if (iterator == list.end()) {
        return push_back(item);
    }
    uint32_t idx = (uint32_t)(iterator - list.begin());
    uint32_t hash = (uint32_t)hash_func(item);
    auto range = index_table.equal_range(hash);
    for (auto it = range.first; it != range.second; it++) {
        if (equal_func(list[it->second], item)) {
            return false;
        }
    }
    for (auto& p : index_table) {
        if (p.second >= idx) {
            p.second++;
        }
    }
    index_table.insert({hash, idx});
    list.insert(iterator, item);
    return true;
}

template <typename T, typename HashFunc, typename EqualFunc>
bool hash_vector<T, HashFunc, EqualFunc>::insert(typename std::vector<T>::const_iterator iterator,
                                                 typename std::vector<T>::const_iterator iterator_begin,
                                                 typename std::vector<T>::const_iterator iterator_end) {
    if (iterator < list.begin() || iterator > list.end()) {
        return false;
    }
    for (auto it = iterator_begin; it != iterator_end; it++) {
        auto it0 = find(*it);
        if (it0 != list.end()) {
            return false;
        }
    }
    uint32_t idx = (uint32_t)(iterator - list.begin());
    for (auto& p : index_table) {
        if (p.second >= idx) {
            p.second += (uint32_t)(iterator_end - iterator_begin);
        }
    }
    for (auto it = iterator_begin; it != iterator_end; it++) {
        uint32_t hash = (uint32_t)hash_func(*it);
        index_table.insert({hash, idx + (uint32_t)(it - iterator_begin)});
    }
    list.insert(iterator, iterator_begin, iterator_end);
    return true;
}

template <typename T, typename HashFunc, typename EqualFunc>
void hash_vector<T, HashFunc, EqualFunc>::erase(typename std::vector<T>::const_iterator iterator) {
    if (iterator < list.begin() || iterator >= list.end()) {
        return;
    }
    uint32_t idx = (uint32_t)(iterator - list.begin());
    uint32_t hash = (uint32_t)hash_func(*iterator);
    auto range = index_table.equal_range(hash);
    for (auto it = range.first; it != range.second; it++) {
        if (it->second == idx) {
            index_table.erase(it);
            break;
        }
    }
    for (auto& p : index_table) {
        if (p.second > idx) {
            p.second--;
        }
    }
    list.erase(iterator);
}

template <typename T, typename HashFunc, typename EqualFunc>
void hash_vector<T, HashFunc, EqualFunc>::erase(typename std::vector<T>::const_iterator iterator_begin,
                                                typename std::vector<T>::const_iterator iterator_end) {
    if (iterator_begin < list.begin() || iterator_end > list.end()
        || iterator_begin >= iterator_end) {
        return;
    }
    uint32_t idx_begin = (uint32_t)(iterator_begin - list.begin());
    uint32_t idx_end = (uint32_t)(iterator_end - list.begin());
    for (uint32_t i = idx_begin; i < idx_end; i++) {
        uint32_t hash = (uint32_t)hash_func(list[i]);
        auto range = index_table.equal_range(hash);
        for (auto it = range.first; it != range.second; it++) {
            if (it->second == i) {
                index_table.erase(it);
                break;
            }
        }
    }
    for (auto& p : index_table) {
        if (p.second >= idx_end) {
            p.second -= (idx_end - idx_begin);
        }
    }
    list.erase(iterator_begin, iterator_end);
}

template <typename T, typename HashFunc, typename EqualFunc>
void hash_vector<T, HashFunc, EqualFunc>::clear() {
    list.clear();
    index_table.clear();
}

template <typename T, typename HashFunc, typename EqualFunc>
typename std::vector<T>::const_iterator hash_vector<T, HashFunc, EqualFunc>::find(const T& item) const {
    uint32_t hash = (uint32_t)hash_func(item);
    auto range = index_table.equal_range(hash);
    for (auto it = range.first; it != range.second; it++) {
        if (equal_func(list[it->second], item)) {
            return list.cbegin() + it->second;
        }
    }
    return list.cend();
}

template <typename T, typename HashFunc, typename EqualFunc>
bool hash_vector<T, HashFunc, EqualFunc>::replace(typename std::vector<T>::const_iterator iterator, const T &item) {
    if (iterator < list.begin() || iterator >= list.end()) {
        return false;
    }
    uint32_t idx = (uint32_t)(iterator - list.begin());
    if (equal_func(*iterator, item)) {
        list[idx] = item;
        return true;
    }
    uint32_t old_hash = (uint32_t)hash_func(list[idx]);
    uint32_t new_hash = (uint32_t)hash_func(item);
    auto range_new = index_table.equal_range(new_hash);
    for (auto it = range_new.first; it != range_new.second; it++) {
        if (equal_func(list[it->second], item)) {
            return false;
        }
    }
    auto range_old = index_table.equal_range(old_hash);
    for (auto it = range_old.first; it != range_old.second; it++) {
        if (it->second == idx) {
            index_table.erase(it);
            break;
        }
    }
    index_table.insert({new_hash, idx});
    list[idx] = item;
    return true;
}