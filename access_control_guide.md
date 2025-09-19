# C++ 类和函数访问控制完整指南

## 1. 基本访问修饰符

### public（公有）
- **访问范围**：任何地方都可以访问
- **用途**：类的对外接口
- **示例**：
```cpp
class MyClass {
public:
    int publicVar;           // 公有变量
    void publicFunction();   // 公有函数
};
```

### private（私有）
- **访问范围**：只有类内部和友元可以访问
- **用途**：内部实现细节，数据封装
- **示例**：
```cpp
class MyClass {
private:
    int privateVar;          // 私有变量
    void privateFunction();  // 私有函数
};
```

### protected（保护）
- **访问范围**：类内部、派生类和友元可以访问
- **用途**：继承体系中的共享成员
- **示例**：
```cpp
class MyClass {
protected:
    int protectedVar;        // 保护变量
    void protectedFunction(); // 保护函数
};
```

## 2. 继承中的访问控制

### 继承类型对访问性的影响

| 基类成员 | public继承 | protected继承 | private继承 |
|---------|-----------|--------------|------------|
| public | public | protected | private |
| protected | protected | protected | private |
| private | 不可访问 | 不可访问 | 不可访问 |

### 示例说明
```cpp
class Base {
public:
    int publicBase;
protected:
    int protectedBase;
private:
    int privateBase;
};

// public 继承
class PublicDerived : public Base {
    // publicBase 仍然是 public
    // protectedBase 仍然是 protected
    // privateBase 不可访问
};

// protected 继承
class ProtectedDerived : protected Base {
    // publicBase 变成 protected
    // protectedBase 仍然是 protected
    // privateBase 不可访问
};

// private 继承
class PrivateDerived : private Base {
    // publicBase 变成 private
    // protectedBase 变成 private
    // privateBase 不可访问
};
```

## 3. 友元（Friend）

### 友元函数
- 可以访问类的所有成员（包括 private 和 protected）
- 不是类的成员函数
- 在类中声明，在类外定义

```cpp
class MyClass {
private:
    int privateData;
public:
    friend void friendFunction(MyClass& obj);
};

void friendFunction(MyClass& obj) {
    obj.privateData = 100;  // 可以访问 private 成员
}
```

### 友元类
- 整个类都是友元
- 友元类的所有成员函数都可以访问该类的所有成员

```cpp
class MyClass {
private:
    int privateData;
public:
    friend class FriendClass;
};

class FriendClass {
public:
    void accessPrivate(MyClass& obj) {
        obj.privateData = 200;  // 可以访问 private 成员
    }
};
```

## 4. 静态成员访问控制

### 静态成员同样遵循访问控制规则
```cpp
class StaticDemo {
public:
    static int publicStatic;
    static void publicStaticFunction();
private:
    static int privateStatic;
    static void privateStaticFunction();
};
```

## 5. 命名空间访问控制

### 通过命名空间控制模块的对外接口
```cpp
namespace MyModule {
    // 公开接口
    class PublicAPI { /* ... */ };
    void publicFunction();
    
    // 内部实现（不导出）
    class InternalImpl { /* ... */ };
    void internalFunction();
}
```

## 6. 访问控制最佳实践

### 数据封装原则
1. **将数据成员设为 private**
2. **提供 public 的 getter/setter 方法**
3. **在 setter 中添加验证逻辑**

```cpp
class Point3D {
private:
    double x_, y_, z_;
    
public:
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
    
private:
    // 内部验证方法
    bool isValidCoordinate(double coord) const {
        return !std::isnan(coord) && !std::isinf(coord);
    }
};
```

### 接口设计原则
1. **最小化 public 接口**
2. **使用 protected 为派生类提供扩展点**
3. **将实现细节隐藏在 private 中**
4. **合理使用友元，但不要滥用**

## 7. 访问控制总结

| 访问修饰符 | 类内部 | 派生类 | 外部 | 友元 |
|-----------|-------|-------|------|------|
| public | ✅ | ✅ | ✅ | ✅ |
| protected | ✅ | ✅ | ❌ | ✅ |
| private | ✅ | ❌ | ❌ | ✅ |

## 8. 实际应用建议

### 在你的 3D 重建项目中
```cpp
class PointCloudProcessor {
public:
    // 对外接口
    void processPointCloud(const PointCloud& cloud);
    void saveResult(const std::string& filename);
    
protected:
    // 派生类可以重写的方法
    virtual void preprocessPointCloud(PointCloud& cloud);
    virtual void postprocessPointCloud(PointCloud& cloud);
    
private:
    // 内部实现细节
    void filterOutliers(PointCloud& cloud);
    void calculateNormals(PointCloud& cloud);
    void optimizeMesh(PointCloud& cloud);
    
    // 内部数据
    double filterThreshold_;
    int optimizationIterations_;
};
```

这样的设计确保了：
- 外部用户只能使用公开的接口
- 派生类可以扩展预处理和后处理逻辑
- 内部实现细节被完全隐藏
- 代码具有良好的可维护性和可扩展性