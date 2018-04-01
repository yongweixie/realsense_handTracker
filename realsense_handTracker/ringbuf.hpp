#pragma once

template <typename T>
class ringbuffer
{
public:
	ringbuffer(size_t size) {
		data = new T[size]();
		_sz = size;
	}
	ringbuffer() {
		delete T[];
		_sz = 0;
	}
	void clean() {
		for (size_t i = 0; i < _sz; i++)
			data[i] = T();
		pos = 0;
	}
	void resize(size_t size) {
		delete T[];
		data = new T[size]();
		_sz = size;
	}
	size_t size() {
		return _sz;
	}
	void push_back(T& src) {
		data[pos] = src;
		pos++;
		if (pos == _sz)pos = 0;
	}
	void setTo(T& src) {
		for (size_t i = 0; i < _sz; i++)
			data[i] = src;
	}
	T& begin() { return data[pos]; }
	T& end() { return data[pos ? pos - 1 : _sz - 1]; }
	T* data;
	T& operator[] (size_t i) {
		return data[truepos(i)];
	}
private:
	inline int truepos(size_t i) {
		auto _i = pos + i;
		return _i < _sz ? _i : _i - _sz;
	}
	size_t pos, _sz;

};
