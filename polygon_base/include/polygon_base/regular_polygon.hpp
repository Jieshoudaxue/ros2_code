#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace polygon_base {

// 整个 RegularPolygon 类是一个抽象基类（ABC），由于存在纯虚函数，不能直接实例化，因此也叫接口类
class RegularPolygon {
public:
    // = 0: 表示纯虚函数，必须需要在派生（子）类中被实现。
    virtual void init(double side_length) = 0;
    virtual double area() = 0;
    // 虚析构函数。在基类中声明虚析构函数是为了确保当通过基类指针删除派生类对象时能够调用正确的析构函数，从而避免资源泄漏。
    virtual ~RegularPolygon(){}

// protected: 表明下面的成员函数和构造函数只能在类内部及继承它的子类中被访问。
protected:
    // 受保护的默认构造函数。由于是受保护的，这个构造函数不能被类的外部直接调用，但可以在派生类中被调用。
    // 它被定义为受保护是为了防止直接创建 RegularPolygon 类型的对象，因为它是一个抽象基类。
    RegularPolygon(){}
};

}   // namespace polygon_base

#endif // POLYGON_BASE_REGULAR_POLYGON_HPP