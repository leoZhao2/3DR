#include <iostream>
#include <string>
#include <vector>
#include <cmath>

// ==================== 1. 基本访问修饰符 ====================
class BasicAccessControl {
public:
    // public 成员 - 任何地方都可以访问
    int publicVar = 10;
    void publicFunction() {
        std::cout << "Public 函数被调用" << std::endl;
    }
    
    // 构造函数和析构函数通常是 public
    BasicAccessControl() {
        std::cout << "构造函数被调用" << std::endl;
    }
    
    ~BasicAccessControl() {
        std::cout << "析构函数被调用" << std::endl;
    }

private:
    // private 成员 - 只有类内部和友元可以访问
    int privateVar = 20;
    void privateFunction() {
        std::cout << "Private 函数被调用" << std::endl;
    }

protected:
    // protected 成员 - 类内部、派生类和友元可以访问
    int protectedVar = 30;
    void protectedFunction() {
        std::cout << "Protected 函数被调用" << std::endl;
    }

public:
    // 通过 public 函数访问 private 成员
    void accessPrivateMembers() {
        std::cout << "通过 public 函数访问 private 变量: " << privateVar << std::endl;
        privateFunction();
    }
    
    // 通过 public 函数访问 protected 成员
    void accessProtectedMembers() {
        std::cout << "通过 public 函数访问 protected 变量: " << protectedVar << std::endl;
        protectedFunction();
    }
};

// ==================== 2. 继承中的访问控制 ====================
class BaseClass {
public:
    int publicBase = 1;
    void publicBaseFunction() {
        std::cout << "Base public 函数" << std::endl;
    }

protected:
    int protectedBase = 2;
    void protectedBaseFunction() {
        std::cout << "Base protected 函数" << std::endl;
    }

private:
    int privateBase = 3;
    void privateBaseFunction() {
        std::cout << "Base private 函数" << std::endl;
    }
};

// public 继承
class PublicDerived : public BaseClass {
public:
    void testAccess() {
        publicBaseFunction();      // ✅ 可以访问
        protectedBaseFunction();   // ✅ 可以访问
        // privateBaseFunction();  // ❌ 不能访问 private
        std::cout << "Public 继承 - public: " << publicBase << ", protected: " << protectedBase << std::endl;
    }
};

// protected 继承
class ProtectedDerived : protected BaseClass {
public:
    void testAccess() {
        publicBaseFunction();      // ✅ 可以访问
        protectedBaseFunction();   // ✅ 可以访问
        std::cout << "Protected 继承 - public: " << publicBase << ", protected: " << protectedBase << std::endl;
    }
};

// private 继承
class PrivateDerived : private BaseClass {
public:
    void testAccess() {
        publicBaseFunction();      // ✅ 可以访问
        protectedBaseFunction();   // ✅ 可以访问
        std::cout << "Private 继承 - public: " << publicBase << ", protected: " << protectedBase << std::endl;
    }
};

// ==================== 3. 友元函数和友元类 ====================
class FriendDemo {
private:
    int privateData = 100;
    
public:
    // 声明友元函数
    friend void friendFunction(FriendDemo& obj);
    
    // 声明友元类
    friend class FriendClass;
    
    void showData() {
        std::cout << "Private data: " << privateData << std::endl;
    }
};

// 友元函数定义
void friendFunction(FriendDemo& obj) {
    std::cout << "友元函数访问 private 数据: " << obj.privateData << std::endl;
    obj.privateData = 200; // 可以修改 private 数据
}

// 友元类定义
class FriendClass {
public:
    void accessPrivateData(FriendDemo& obj) {
        std::cout << "友元类访问 private 数据: " << obj.privateData << std::endl;
        obj.privateData = 300;
    }
};

// ==================== 4. 静态成员访问控制 ====================
class StaticAccessControl {
public:
    static int publicStaticVar;
    static void publicStaticFunction() {
        std::cout << "Public static 函数" << std::endl;
    }

private:
    static int privateStaticVar;
    static void privateStaticFunction() {
        std::cout << "Private static 函数" << std::endl;
    }

public:
    static void accessPrivateStatic() {
        std::cout << "通过 public 函数访问 private static: " << privateStaticVar << std::endl;
        privateStaticFunction();
    }
};

// 静态成员定义
int StaticAccessControl::publicStaticVar = 100;
int StaticAccessControl::privateStaticVar = 200;

// ==================== 5. 命名空间访问控制 ====================
namespace MyNamespace {
    // 公开的接口
    class PublicAPI {
    public:
        void publicMethod() {
            std::cout << "命名空间中的 public 方法" << std::endl;
        }
    };
    
    // 内部实现细节（不导出）
    class InternalImplementation {
    private:
        int internalData = 500;
    public:
        void internalMethod() {
            std::cout << "内部实现方法" << std::endl;
        }
    };
    
    // 公开函数
    void publicFunction() {
        std::cout << "命名空间中的 public 函数" << std::endl;
    }
    
    // 内部函数（不导出）
    void internalFunction() {
        std::cout << "内部函数" << std::endl;
    }
}

// ==================== 6. 访问控制最佳实践示例 ====================
class Point3D {
private:
    double x_, y_, z_;
    
public:
    // 构造函数
    Point3D(double x = 0, double y = 0, double z = 0) : x_(x), y_(y), z_(z) {}
    
    // Getter 方法（只读访问）
    double getX() const { return x_; }
    double getY() const { return y_; }
    double getZ() const { return z_; }
    
    // Setter 方法（受控的写访问）
    void setX(double x) { 
        if (isValidCoordinate(x)) {
            x_ = x; 
        }
    }
    
    void setY(double y) { 
        if (isValidCoordinate(y)) {
            y_ = y; 
        }
    }
    
    void setZ(double z) { 
        if (isValidCoordinate(z)) {
            z_ = z; 
        }
    }
    
    // 公共接口方法
    double distanceFromOrigin() const {
        return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
    }
    
    void print() const {
        std::cout << "Point3D(" << x_ << ", " << y_ << ", " << z_ << ")" << std::endl;
    }
    
private:
    // 内部验证方法
    bool isValidCoordinate(double coord) const {
        return !std::isnan(coord) && !std::isinf(coord);
    }
};

// ==================== 主函数测试 ====================
int main() {
    std::cout << "=== C++ 访问控制演示 ===" << std::endl;
    
    // 1. 基本访问控制测试
    std::cout << "\n1. 基本访问控制:" << std::endl;
    BasicAccessControl obj;
    obj.publicFunction();           // ✅ 可以访问
    obj.accessPrivateMembers();     // ✅ 通过 public 函数访问 private
    obj.accessProtectedMembers();   // ✅ 通过 public 函数访问 protected
    
    // 2. 继承访问控制测试
    std::cout << "\n2. 继承访问控制:" << std::endl;
    PublicDerived pubDerived;
    pubDerived.testAccess();
    pubDerived.publicBaseFunction(); // ✅ public 继承可以访问
    
    ProtectedDerived protDerived;
    protDerived.testAccess();
    // protDerived.publicBaseFunction(); // ❌ protected 继承外部不能访问
    
    PrivateDerived privDerived;
    privDerived.testAccess();
    // privDerived.publicBaseFunction(); // ❌ private 继承外部不能访问
    
    // 3. 友元测试
    std::cout << "\n3. 友元访问控制:" << std::endl;
    FriendDemo friendObj;
    friendObj.showData();
    friendFunction(friendObj);      // ✅ 友元函数可以访问 private
    friendObj.showData();
    
    FriendClass friendClassObj;
    friendClassObj.accessPrivateData(friendObj); // ✅ 友元类可以访问 private
    friendObj.showData();
    
    // 4. 静态成员访问控制测试
    std::cout << "\n4. 静态成员访问控制:" << std::endl;
    std::cout << "Public static var: " << StaticAccessControl::publicStaticVar << std::endl;
    StaticAccessControl::publicStaticFunction();
    StaticAccessControl::accessPrivateStatic();
    
    // 5. 命名空间访问控制测试
    std::cout << "\n5. 命名空间访问控制:" << std::endl;
    MyNamespace::PublicAPI api;
    api.publicMethod();
    MyNamespace::publicFunction();
    
    // 6. 最佳实践示例
    std::cout << "\n6. 最佳实践示例:" << std::endl;
    Point3D point(1, 2, 3);
    point.print();
    std::cout << "距离原点: " << point.distanceFromOrigin() << std::endl;
    
    // 通过 setter 修改坐标
    point.setX(4);
    point.setY(5);
    point.setZ(6);
    point.print();
    
    return 0;
}