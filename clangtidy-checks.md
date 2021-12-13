# List of enabled clang-tidy checks

## For clangtidy-autofix (via .clang-tidy file)
This rules will apply fixes automatically (if possible).

```
modernize-deprecated-ios-base-aliases
modernize-loop-convert
modernize-make-shared
modernize-make-unique
modernize-pass-by-value
modernize-redundant-void-arg
modernize-replace-auto-ptr
modernize-use-auto
modernize-use-bool-literals
modernize-use-default-member-init
modernize-use-emplace
modernize-use-equals-default
modernize-use-equals-delete
modernize-use-nodiscard
modernize-use-nullptr
modernize-use-override
modernize-use-using
readability-braces-around-statements
readability-const-return-type
readability-container-size-empty
readability-delete-null-pointer
readability-identifier-naming
readability-inconsistent-declaration-parameter-name
readability-make-member-function-const
readability-non-const-parameter
readability-qualified-auto
readability-redundant-declaration
readability-redundant-function-ptr-dereference
readability-redundant-member-init
readability-redundant-smartptr-get
readability-redundant-string-cstr
readability-redundant-string-init
readability-simplify-boolean-expr
readability-simplify-subscript-expr
readability-string-compare
readability-uniqueptr-delete-release
```

## For clangtidy-check (manual evaluation)
These checks can be run on demand and might be controversial in come cases.

The list contains all checks available in clang 12. Unwanted checks are marked
with `##` and check, which already are in the .clang-tidy file are marked with
`#`. Each line also has information, if the check has auto-fixes available.

```
bugprone-*
-bugprone-easily-swappable-parameters

## cppcoreguidelines-avoid-goto 	 No
cppcoreguidelines-avoid-non-const-global-variables 	 No
cppcoreguidelines-init-variables 	Yes
cppcoreguidelines-interfaces-global-init 	 No
## cppcoreguidelines-macro-usage 	 No
cppcoreguidelines-narrowing-conversions 	 No
cppcoreguidelines-no-malloc 	 No
## cppcoreguidelines-owning-memory 	 No
cppcoreguidelines-prefer-member-initializer 	Yes
cppcoreguidelines-pro-bounds-array-to-pointer-decay 	 No
## cppcoreguidelines-pro-bounds-constant-array-index 	Yes
## cppcoreguidelines-pro-bounds-pointer-arithmetic 	 No
cppcoreguidelines-pro-type-const-cast 	 No
cppcoreguidelines-pro-type-cstyle-cast 	Yes
cppcoreguidelines-pro-type-member-init 	Yes
cppcoreguidelines-pro-type-reinterpret-cast 	 No
cppcoreguidelines-pro-type-static-cast-downcast 	Yes
## cppcoreguidelines-pro-type-union-access 	 No
## cppcoreguidelines-pro-type-vararg 	 No
cppcoreguidelines-slicing 	 No
cppcoreguidelines-special-member-functions 	 No
cppcoreguidelines-virtual-class-destructor 	Yes

misc-definitions-in-headers 	Yes
## misc-misleading-identifier 	 No
misc-misplaced-const 	 No
misc-new-delete-overloads 	 No
## misc-no-recursion 	 No
misc-non-copyable-objects 	 No
misc-non-private-member-variables-in-classes 	 No
misc-redundant-expression 	Yes
misc-static-assert 	Yes
misc-throw-by-value-catch-by-reference 	 No
misc-unconventional-assign-operator 	 No
misc-uniqueptr-reset-release 	Yes
misc-unused-alias-decls 	Yes
misc-unused-parameters 	Yes
misc-unused-using-decls 	Yes

## modernize-avoid-bind 	Yes
## modernize-avoid-c-arrays 	 No
modernize-concat-nested-namespaces 	Yes
## modernize-deprecated-headers 	Yes
# modernize-deprecated-ios-base-aliases 	Yes
# modernize-loop-convert 	Yes
# modernize-make-shared 	Yes
# modernize-make-unique 	Yes
# modernize-pass-by-value 	Yes
## modernize-raw-string-literal 	Yes
# modernize-redundant-void-arg 	Yes
modernize-replace-auto-ptr 	Yes
modernize-replace-disallow-copy-and-assign-macro 	Yes
modernize-replace-random-shuffle 	Yes
## modernize-return-braced-init-list 	Yes
modernize-shrink-to-fit 	Yes
modernize-unary-static-assert 	Yes
# modernize-use-auto 	Yes
# modernize-use-bool-literals 	Yes
# modernize-use-default-member-init 	Yes
# modernize-use-emplace 	Yes
# modernize-use-equals-default 	Yes
# modernize-use-equals-delete 	Yes
# modernize-use-nodiscard 	Yes
## modernize-use-noexcept 	Yes
# modernize-use-nullptr 	Yes
# modernize-use-override 	Yes
## modernize-use-trailing-return-type 	Yes
modernize-use-transparent-functors 	Yes
modernize-use-uncaught-exceptions 	Yes
# modernize-use-using 	Yes

performance-faster-string-find 	Yes
performance-for-range-copy 	Yes
performance-implicit-conversion-in-loop 	 No
performance-inefficient-algorithm 	Yes
performance-inefficient-string-concatenation 	 No
performance-inefficient-vector-operation 	Yes
performance-move-const-arg 	Yes
performance-move-constructor-init 	 No
performance-no-automatic-move 	 No
performance-no-int-to-ptr 	 No
performance-noexcept-move-constructor 	Yes
performance-trivially-destructible 	Yes
performance-type-promotion-in-math-fn 	Yes
performance-unnecessary-copy-initialization 	 No
performance-unnecessary-value-param 	Yes

readability-avoid-const-params-in-decls 	 No
# readability-braces-around-statements 	Yes
# readability-const-return-type 	Yes
# readability-container-data-pointer 	Yes
readability-container-size-empty 	Yes
readability-convert-member-functions-to-static 	 No
# readability-delete-null-pointer 	Yes
## readability-else-after-return 	Yes
## readability-function-cognitive-complexity 	 No
## readability-function-size 	 No
## readability-identifier-length 	 No
## readability-identifier-naming 	Yes
## readability-implicit-bool-conversion 	Yes
# readability-inconsistent-declaration-parameter-name 	Yes
## readability-isolate-declaration 	Yes
## readability-magic-numbers 	 No
# readability-make-member-function-const 	Yes
readability-misleading-indentation 	 No
readability-misplaced-array-index 	Yes
readability-named-parameter 	Yes
readability-non-const-parameter 	Yes
# readability-qualified-auto 	Yes
readability-redundant-access-specifiers 	Yes
readability-redundant-control-flow 	Yes
# readability-redundant-declaration 	Yes
# readability-redundant-function-ptr-dereference 	Yes
# readability-redundant-member-init 	Yes
readability-redundant-preprocessor 	 No
# readability-redundant-smartptr-get 	Yes
# readability-redundant-string-cstr 	Yes
# readability-redundant-string-init 	Yes
# readability-simplify-boolean-expr 	Yes
readability-simplify-subscript-expr 	Yes
readability-static-accessed-through-instance 	Yes
readability-static-definition-in-anonymous-namespace 	Yes
# readability-string-compare 	Yes
readability-suspicious-call-argument 	 No
# readability-uniqueptr-delete-release 	Yes
readability-uppercase-literal-suffix 	Yes
```
