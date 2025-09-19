#include <iostream>

class AccessRestrictions {
public:
    int publicVar = 10;
    void publicFunction() {
        std::cout << "Public function" << std::endl;
    }

private:
    int privateVar = 20;
    void privateFunction() {
        std::cout << "Private function" << std::endl;
    }

protected:
    int protectedVar = 30;
    void protectedFunction() {
        std::cout << "Protected function" << std::endl;
    }
};

class DerivedClass : public AccessRestrictions {
public:
    void testDerivedAccess() {
        // 在派生类中可以访问
        publicFunction();      // ✅ 可以访问
        protectedFunction();   // ✅ 可以访问
        std::cout << "Public var: " << publicVar << std::endl;
        std::cout << "Protected var: " << protectedVar << std::endl;
        
        // privateFunction();  // ❌ 不能访问 private
        // std::cout << privateVar; // ❌ 不能访问 private
    }
};

int main() {
    AccessRestrictions obj;
    
    // 可以访问 public 成员
    obj.publicFunction();
    std::cout << "Public var: " << obj.publicVar << std::endl;
    
    // 以下代码如果取消注释会导致编译错误：
    
    // obj.privateFunction();     // ❌ 编译错误：'void AccessRestrictions::privateFunction()' is private
    // std::cout << obj.privateVar; // ❌ 编译错误：'int AccessRestrictions::privateVar' is private
    // obj.protectedFunction();   // ❌ 编译错误：'void AccessRestrictions::protectedFunction()' is protected
    // std::cout << obj.protectedVar; // ❌ 编译错误：'int AccessRestrictions::protectedVar' is protected
    
    // 派生类测试
    DerivedClass derived;
    derived.testDerivedAccess();
    
    return 0;
}