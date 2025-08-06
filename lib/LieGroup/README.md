





Goal:
- Try to put as much template programming as possible

Can it be just a wrapper? Then it does not even need to copy that data.




## Interface

For Lie group, the class should support
- identity
- inverse operation
- multiplication

```c++
template <std::floating_point T> class Group {
    using DataType = ...;

  public:
    // constructors
    
    // access underlying data
    DataType &data() { return data_; }

    const DataType &data() const { return data_; }

    // operator
    Group operator*(const Group &other) const;
    Group inv() const;

  private:
    DataType data_;
};
```
