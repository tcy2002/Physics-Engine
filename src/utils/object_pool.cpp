template <typename T, size_t BlockSize>
void* ObjectPool<T, BlockSize>::aligned_malloc(size_t size, int align) {
    const int pointer_size = sizeof(void*);
    const int requested_size = (int)size + align - 1 + pointer_size;
    void* raw = malloc(requested_size);
    uintptr_t start = (uintptr_t)raw + pointer_size;
    void* aligned = (void*)((start + align - 1) & ~(align - 1));
    *(void**)((uintptr_t)aligned - pointer_size) = raw;
    return aligned;
}

template <typename T, size_t BlockSize>
void ObjectPool<T, BlockSize>::aligned_free(T* ptr) {
    if (ptr) {
        free(((T**)ptr)[-1]);
    }
}

template <typename T, size_t BlockSize>
T* ObjectPool<T, BlockSize>::allocate() {
    if (_free_node == nullptr) {
        allocBlock();
    }
    T* ptr = reinterpret_cast<T*>(_free_node);
    _free_node = _free_node->next;
    return ptr;
}

template <typename T, size_t BlockSize>
void ObjectPool<T, BlockSize>::deallocate(T* ptr) {
    if (ptr) {
        pushFreeNode(reinterpret_cast<FreeNode*>(ptr));
    }
}

template <typename T, size_t BlockSize>
void ObjectPool<T, BlockSize>::pushFreeNode(FreeNode *ptr) {
    if (ptr) {
        ptr->next = _free_node;
        _free_node = ptr;
    }
}

template <typename T, size_t BlockSize>
void ObjectPool<T, BlockSize>::allocBlock() {
    T* block = reinterpret_cast<T*>(aligned_malloc(BlockSize, alignof(T)));
    _blocks.push_back((void*)block);
    T* it = block + (BlockSize / sizeof(T));
    while(--it >= block) {
        pushFreeNode(reinterpret_cast<FreeNode*>(it));
    }
}

template <typename T, size_t BlockSize>
void ObjectPool<T, BlockSize>::destroyAll() {
    std::vector<T*> free_nodes;
    FreeNode* iter = _free_node;
    while (iter) {
        free_nodes.push_back(reinterpret_cast<T*>(iter));
        iter = iter->next;
    }

    // TODO: check if this is necessary
    if (free_nodes.empty()) {
        return;
    }

    static auto cmp_t_ptr = [=](T* i, T* j) { return i < j; };
    static auto cmp_v_ptr = [=](void* i, void* j) { return i < j; };
    std::sort(free_nodes.begin(), free_nodes.end(), cmp_t_ptr);
    std::sort(_blocks.begin(), _blocks.end(), cmp_v_ptr);

    int j = 0;
    for (int i = 0; i < _blocks.size(); i++) {
        T* elem = reinterpret_cast<T*>(_blocks[i]);
        T* b_end = elem + (BlockSize / sizeof(T));
        for(; elem < b_end; elem++){
            if (free_nodes[j] == elem) {
                j++;
            } else if (free_nodes[j] > elem) {
                elem->~T();
            } else {
                // TODO: check why this happens and why should not happen
                //PE_LOG_ERROR << "Pool destroy error" << std::endl;
            }

            // TODO: check if this is necessary
            if (j >= free_nodes.size()) {
                return;
            }
        }
    }
}

template <typename T, size_t BlockSize>
ObjectPool<T, BlockSize>::ObjectPool(): _free_node(nullptr) {
    if (BlockSize < sizeof(T)) {
        PE_LOG_ERROR << "Pool size too low (" << BlockSize << ")" << PE_ENDL;
        exit(-1);
    }
}

template <typename T, size_t BlockSize>
ObjectPool<T, BlockSize>::~ObjectPool() {
    destroyAll();
    for (auto p : _blocks) {
        aligned_free(reinterpret_cast<T*>(p));
    }
}

template<typename T, size_t BlockSize>
template<typename... Args>
T *ObjectPool<T, BlockSize>::create(Args &&... args) {
    T* t = allocate();
    return t ? new (t) T(std::forward<Args>(args)...) : nullptr;
}

template <typename T, size_t BlockSize>
void ObjectPool<T, BlockSize>::destroy(T* ptr) {
    if (ptr) {
        ptr->~T();
        deallocate(ptr);
    }
}
