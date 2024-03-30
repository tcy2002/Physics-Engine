template <typename T, typename HashFunc, typename EqualFunc>
bool HashArray<T, HashFunc, EqualFunc>::contains(const T& item) const {
    uint32_t hash = _hash_func(item);
    auto range = _table.equal_range(hash);
    for (auto it = range.first; it != range.second; it++) {
        if (_equal_func(_list[it->second], item)) {
            return true;
        }
    }
    return false;
}

template <typename T, typename HashFunc, typename EqualFunc>
void HashArray<T, HashFunc, EqualFunc>::push_back(const T& item) {
    uint32_t hash = _hash_func(item);
    _table.insert({hash, (int)size()});
    _list.push_back(item);
}

template <typename T, typename HashFunc, typename EqualFunc>
void HashArray<T, HashFunc, EqualFunc>::pop_back() {
    if (empty()) return;
    int idx = (int)size() - 1;
    uint32_t hash = _hash_func(_list[idx]);
    auto range = _table.equal_range(hash);
    for (auto it = range.first; it != range.second; it++) {
        if (it->second == idx) {
            _table.erase(it);
            break;
        }
    }
    _list.pop_back();
}

template <typename T, typename HashFunc, typename EqualFunc>
void HashArray<T, HashFunc, EqualFunc>::insert(typename std::vector<T>::iterator iterator, const T& item) {
    if (iterator > _list.end()) {
        return;
    } else if (iterator == _list.end()) {
        push_back(item);
        return;
    }
    int idx = (int)(iterator - _list.begin());
    uint32_t hash = _hash_func(item);
    for (auto& it : _table) {
        if (it.second >= idx) {
            it.second++;
        }
    }
    _table.insert({hash, idx});
    _list.insert(iterator, item);
}

template <typename T, typename HashFunc, typename EqualFunc>
void HashArray<T, HashFunc, EqualFunc>::insert(typename std::vector<T>::iterator iterator,
                                               const std::vector<T>& items) {
    if (iterator > _list.end()) {
        return;
    } else if (iterator == _list.end()) {
        for (const auto& item : items) {
            push_back(item);
        }
        return;
    }
    int idx = (int)(iterator - _list.begin());
    for (auto it = items.rbegin(); it != items.rend(); it++) {
        insert(_list.begin() + idx, *it);
    }
}

template <typename T, typename HashFunc, typename EqualFunc>
void HashArray<T, HashFunc, EqualFunc>::erase(typename std::vector<T>::iterator iterator) {
    if (empty() || iterator >= _list.end()) return;
    if (iterator == _list.end() - 1) {
        pop_back();
        return;
    }
    int idx = (int)(iterator - _list.begin());
    uint32_t hash = _hash_func(_list[idx]);
    auto range = _table.equal_range(hash);
    for (auto it = range.first; it != range.second; it++) {
        if (it->second == idx) {
            _table.erase(it);
            break;
        }
    }
    for (auto& it : _table) {
        if (it.second > idx) {
            it.second--;
        }
    }
    _list.erase(iterator);
}

template <typename T, typename HashFunc, typename EqualFunc>
void HashArray<T, HashFunc, EqualFunc>::erase(typename std::vector<T>::iterator it_begin,
                                              typename std::vector<T>::iterator it_end) {
    int idx_begin = (int)(it_begin - _list.begin());
    int idx_end = (int)(it_end - _list.begin());
    if (idx_end > (int)size()) {
        idx_end = (int)size();
    }
    for (int i = idx_begin; i < idx_end; i++) {
        erase(_list.begin() + idx_begin);
    }
}

template <typename T, typename HashFunc, typename EqualFunc>
typename std::vector<T>::iterator HashArray<T, HashFunc, EqualFunc>::find_first(const T& item) {
    uint32_t hash = _hash_func(item);
    auto range = _table.equal_range(hash);
    for (auto it = range.first; it != range.second; it++) {
        if (_equal_func(_list[it->second], item)) {
            return _list.begin() + it->second;
        }
    }
    return _list.end();
}