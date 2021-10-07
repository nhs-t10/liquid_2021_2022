//if we're in a web worker
if (typeof importScripts === "function") {
    onmessage = function (event) {
        var data = event.data;

        var fns = {
            astToString: function () {
                return postMessage({
                    nonce: data.nonce,
                    data: astToString(data.args[0], data.args[1], data.args[2], undefined, undefined, undefined, data.args[3], data.args[4], "")
                });
            },
            clearVariableRegistry: function () {
                return postMessage({
                    nonce: data.nonce,
                    data: clearVariableRegistry()
                });
            }
        };

        fns[data.function]();
    }
}

if (typeof window === "undefined") var window = {};

if(typeof module !== "undefined") module.exports = { astToString: astToString }

var globalVarRegistry = {};


function clearVariableRegistry() {
    var keys = Object.keys(globalVarRegistry);

    for (var i = 0; i < keys.length; i++) {
        delete globalVarRegistry[keys[i]];
    }

    return true;
}

/**
 * @typedef {Object} StylingMode
 * @property {boolean} javaBracketsStyle Use Java-style brackets, with brackets on the same line. If false, it will go to C-style brackets. 
 * @property {string} indentBy The string to indent blocks by. Must be whitespace.
 * @property {string} linesAfterImport The lines of empty space to put after import statements but before classes.
 * @property {string} spaceAfterStatement Space to include after statements like `for`, but before their parameters.
 * @property {boolean} colorize Whether to colorize the output with HTML.
 * @property {boolean} removeComments Whether to remove comments or not.
 * @property {string} spaceInExpression Space to include inside expressions.
 * @property {boolean} leaveOffFloatSuffix Convert floats to doubles
 * @property {boolean} dontHighlightPairedChars Don't link block openers and parens with their counterpart closers.
 * @property {boolean} hideExplainations Don't display explaination tooltips on select syntax constructs.
 * @property {string} ifElseNewline A whitespace string to put between the ends of `if` statements and their `else` statements
 * @property {('source' | 'block' | 'line')} singleLineBlockBrackets How to treat single-line blocks that can have their brackets removed.
 * @property {boolean} dontRegisterVariables
 */
/**
 * Stringify a Java AST
 * @param {Object} ast The AST to stringify.
 * @param {StylingMode} style How the output should be styled.
 */
function astToString(ast, style, parentScope, nodePath, siblingIndex, address, parent, codeblockId, annotationConnectorId) {
    if (!ast) return "";

    if (parentScope === undefined) parentScope = [];
    if (style === undefined) style = {};
    if (codeblockId === undefined) codeblockId = Math.floor(Math.random() * 1000) + "-" + Date.now();
    if (parent === undefined) parent = {};

    if (style.isSnippet && ast.type == "COMPILATION_UNIT") {
        ast = ast.types[0].declaration.body.declarations[0];
    }

    //copy in order to not modify original
    style = Object.assign({}, style);

    //default to 4-space styling
    if (style.indentBy === undefined) style.indentBy = "    ";

    //default to java-style spacing
    if (style.javaBracketsStyle === undefined) style.javaBracketsStyle = true;
    if (style.spaceAfterStatement === undefined) style.spaceAfterStatement = "";
    if (style.linesAfterImport === undefined) style.linesAfterImport = "\n";
    if (style.removeComments === undefined) style.removeComments = false;
    if (style.spaceInExpression === undefined) style.spaceInExpression = style.spaceAfterStatement || " ";
    if (style.dontHighlightPairedChars === undefined) style.dontHighlightPairedChars = false;
    if (style.leaveOffFloatSuffix === undefined) style.leaveOffFloatSuffix = true;
    if (style.ifElseNewline === undefined) style.ifElseNewline = "\n";
    if (style.singleLineBlockBrackets === undefined) style.singleLineBlockBrackets = "block"


    var bracketTypes = ["\n", " "];

    var isLeaf = 0;

    nodePath = (nodePath === undefined ? [] : nodePath).concat([{
        type: ast.type,
        idx: siblingIndex || 0
    }]);

    address = address === undefined ? [] : address;

    var pairedCharId = 0;
    function createPairedChar(char) {
        if (style.dontHighlightPairedChars) return style.colorize ? encodeCharacterEntities(char) : char;
        if (!style.colorize) return char;

        var inOut = pairedCharId % 2;
        var index = Math.floor(pairedCharId / 2);
        var res = `<span id="codeblock-${codeblockId}-${address.join("-")}-${inOut ? "out" : "in"}-${index}" class="hlast hlast-pairedchar">${encodeCharacterEntities(char)}</span>`;
        pairedCharId++;
        return res;
    }

    var annotationConnectorIdChildIndex = 0;
    function recurse(a, n, sc) {
        if (typeof a === "string") a = [a];

        var newScope = parentScope;
        if (parentScope.real) newScope = parentScope.real;

        if (!style.dontRegisterVariables) {
            var nextScopeComponent = getScopeComponent(ast, address.concat(n ? a : [n]));
            newScope = newScope.concat([nextScopeComponent]);
            createScope(newScope);

            if (sc) sc.real = newScope;
        }

        if (typeof n == "number") annotationConnectorIdChildIndex++;

        if (typeof a === "object" && a.constructor !== Array) return astToString(
            a, style,
            (sc || newScope),
            nodePath, isLeaf, address.concat([n]), ast, codeblockId, annotationConnectorId + "," + (annotationConnectorIdChildIndex));

        //find the object indicated by the path passed in
        var target = ast;
        for (var i = 0; i < a.length; i++) target = target[a[i]];

        if (target) target.__parent__ = ast;


        if (typeof a[a.length - 1] == "number") annotationConnectorIdChildIndex++;
        isLeaf++;

        return astToString(target, 
            style, 
            sc || newScope, 
            nodePath, 
            isLeaf, 
            address.concat(a), 
            ast, 
            codeblockId, 
            annotationConnectorId + "," + (annotationConnectorIdChildIndex));
    }

    function conditionallyRemoveBracketsFromSingleLineBlocks(block) {
        var blockNorm = (block.type == "Block" && block.statements.length == 1) ? block.statements[0] : block;

        //if-in-for, while-in-for, etc. should *always* be blocks for clarity.
        if ((blockNorm.type.includes("FOR") || blockNorm.type.includes("WHILE") || blockNorm.type.includes("IF"))
            && (ast.type.includes("FOR") || ast.type.includes("WHILE") || ast.type.includes("IF"))
            && ast.type != blockNorm.type) return { type: "Block", statements: [blockNorm] };


        if (style.singleLineBlockBrackets == "source" || block.type == "IfStatement" || block.type == "ForStatement" || block.type == "WHILE_STATEMENT") return block;
        else if (block.type != "Block" && (style.singleLineBlockBrackets == "block")) return { type: "Block", statements: [block] };
        else if ((block.statements && block.statements.length == 1) && style.singleLineBlockBrackets == "line") return block.statements[0];
        else return block;
    }

    style.isDense = style.spaceAfterStatement == "dense";
    if (style.isDense) {
        style.spaceAfterStatement = "";
        style.indentBy = "";
    }


    var result = "";

    switch (ast.type || ast.ast_type) {
        case "CompilationUnit":
            result += recurse("package") +
                ast.imports.map(function (x, i) { return recurse(["imports", i]) + ";\n" }).join("") +
                (ast.imports.length ? style.linesAfterImport : "") +
                ast.types.map(function (x, i) { return recurse(["types", i]) }).join(style.spaceBetweenClasses);
            break;
        case "PACKAGE_DECLARATION":
            var packageName = astToString(ast.name, {
                colorize: false,
                dontHighlightPairedChars: true,
                dontRegisterVariables: true
            });
            parentScope.splice(1, 0, packageName);

            result += "package " + recurse("name") + ";\n";
            break;
        case "FieldAccess":
            result += recurse("expression") + "." + recurse("name");
            break;
        case "QualifiedName":
            result += recurse("qualifier") + "." + recurse("name");
            break;
        case "TypeDeclaration":
            var typeyKeyword = ast.interface ? "interface" : "class";

            result += ast.modifiers.map((x, i) => recurse(["modifiers", i])).join("") +
                (style.colorize ? "<span class=\"hlast hlast-keyword\">" + typeyKeyword + "</span> " : typeyKeyword + " ") + recurse("name") +
                (ast.superclassType ? (style.colorize ? "<span class=\"hlast hlast-keyword\"> extends </span>" : " extends ") + recurse("superclassType") : "") +
                (ast.superInterfaceTypes.length ? (style.colorize ? "<span class=\"hlast hlast-keyword\"> implements </span>" : " implements ") + ast.superInterfaceTypes.map((x, i) => recurse(["superInterfaceTypes", i])).join(", ") : "") +
                (bracketTypes[+!!style.javaBracketsStyle]) +
                indent(recurse("bodyDeclarations"), style.indentBy, style.javaBracketsStyle, true); //never indent last line, maybe indent first line depending on bracket style
            break;
        case "COMMENT_STANDALONE":
            if (style.removeComments || style.isDense) break;
            result += indent(undent(ast.value), " ", false, false);
            break;
        case "Modifier":
            result += ast.keyword + " ";
            break;
        case "SimpleName":
            if(!ast.value) ast.value = ast.identifier;

            if (!style.dontRegisterVariables) {
                if (parent.type === "CLASS_DECLARATION") {
                    //register classes
                    var varNameUnformatted = ast.value;

                    var type = parent.extends || "Object";

                    if (!style.dontRegisterVariables) registerVariable(parentScope, varNameUnformatted, type, "class");

                    result += style.colorize ? `<span class="hlast hlast--class-definition-identifier" data-variable-address="${codeblockId}/${encodeCharacterEntities(parentScope.join(""))}">${encodeCharacterEntities(varNameUnformatted)}</span>` : varNameUnformatted;
                    break;
                } else {
                    var varScope = getVariableScope(parentScope, ast.value);
                    if (varScope && style.colorize) result +=
                        `<span class="hlast hlast--variable-reference-identifier ${generateDescribingClasses(ast.value, isLeaf)}" data-variable-scope="${encodeCharacterEntities(varScope.scopeKey)}" data-variable-address="${codeblockId}/${encodeCharacterEntities(varScope.address)}" data-variable-type="${varScope.type}" data-variable-typetype="${encodeCharacterEntities(varScope.typeType && varScope.typeType.value)}">${encodeCharacterEntities(ast.value)}</span>`;
                    else result += ast.value;
                    break;
                }
            }
        case "PRIMITIVE_TYPE":
            break;
        case "BooleanLiteral":
            result += ast.booleanValue;
            break;
        case "StringLiteral":
        case "CharacterLiteral":
            result += style.colorize ? encodeCharacterEntities(ast.escapedValue) : ast.escapedValue;
            break;
        case "NumberLiteral":
            result += style.colorize ? encodeCharacterEntities(ast.token) : ast.token;
            break;
        case "FLOAT_LITERAL":
            result += ast.value;
            if (!style.leaveOffFloatSuffix) result += (style.colorize ? "<span class=\"hlast hlast-float-literal-suffix\">f</span>" : "f");
            break;
        case "TYPE_LIST":
        case "QUALIFIED_NAME_LIST":
            //firefox bug where arrays sometimes appear as empty slots until manually iterated
            for (var i = ast.list.length - 1; i >= 0; i--) result += (i >= 1 ? ", " : "") + recurse(["list", i]);
            break;
        case "CLASS_BODY":
            result += createPairedChar("{") + "\n" +
                ast.declarations.map(function (x, i) { return recurse(["declarations", i]); }).join("\n") + "\n" +
                createPairedChar("}");
            break;
        case "VariableDeclarationExpression":
            result += ast.modifiers.map((x, i) => recurse(["modifiers", i])).join("") +
                recurse("typeType") + " " + 
                ast.fragments.map((x,i)=>recurse(["fragments", i])).join("," + style.spaceInExpression);
            break;
        case "SingleVariableDeclaration":
            result += ast.modifiers.map((x, i) => recurse(["modifiers", i])).join("") +
            recurse("typeType") + "[]".repeat(ast.extraDimensions) +
            " " + recurse("name") +
            (ast.initializer ? style.spaceInExpression + recurse({ type: "Operator", operator: "=" }) + style.spaceInExpression + recurse("initializer") : "");
            break;
        case "VariableDeclarationFragment":
            result += recurse("name") + "[]".repeat(ast.extraDimensions) +
            (ast.initializer ? style.spaceInExpression + recurse({ type: "Operator", operator: "=" }) + style.spaceInExpression + recurse("initializer") : "");
            break;
        case "FieldDeclaration":
        case "VariableDeclarationStatement":
            result += ast.modifiers.map((x, i) => recurse(["modifiers", i])).join("") +
            recurse("typeType") + "[]".repeat(ast.extraDimensions) +
            " " + ast.fragments.map((x,i)=>recurse(["fragments", i])).join("," + style.spaceInExpression) +
            (ast.initializer ? style.spaceInExpression + recurse({ type: "Operator", operator: "=" }) + style.spaceInExpression + recurse("initializer") : "") + ";";
            break;
        case "VARIABLE_DECLARATOR_ID":
            var varNameUnformatted = ast.id.value;

            var typeFullyQualified = "";
            if (!style.dontRegisterVariables) {
                var typeType = parent.typeType || parent.__parent__.typeType || parent.__parent__.__parent__.typeType;

                registerVariable(parentScope, varNameUnformatted, typeType.value, parent.type.toLowerCase());
            }

            var varNameWrapped = style.colorize ? `<span class="hlast hlast--variable-definition-identifier" data-var-typetype="${encodeCharacterEntities(typeFullyQualified)}" data-variable-address="${codeblockId}/${encodeCharacterEntities(parentScope.join("") + "." + varNameUnformatted)}">${varNameUnformatted}</span>` : recurse("id");

            result += varNameWrapped +
                ast.dimensions.map(function (x, i) { return recurse(["dimensions", i]); }).join("");
            break;
        case "CONSTRUCTOR_DECLARATION":
            result += recurse("name") + style.spaceAfterStatement + recurse("parameters") +
                (ast.throws ? " throws " + recurse("throws") : "") +
                bracketTypes[+!!style.javaBracketsStyle] +
                recurse("body");
            break;
        case "MethodDeclaration":
            result += ast.modifiers.map((x, i) => recurse(["modifiers", i])).join("");

            if(ast.typeParameters.length > 0) throw "type parameters used!";
            
            result += recurse("returnType2") + ("[]").repeat(ast.extraDimensions) +
                (ast.constructor ? "" : " ") + recurse("name") + 
                style.spaceAfterStatement +
                createPairedChar("(") + ast.parameters.map((x, i) => recurse(["parameters", i])).join("," + style.spaceInExpression) + createPairedChar(")") +
                (ast.throws ? " throws " + ast.thrownExceptions.map((x,i)=>recurse(["thrownExceptions", i])) : "") +
                bracketTypes[+!!style.javaBracketsStyle] +
                recurse("body");
            break;
        case "FORMAL_PARAMETERS":
            result += createPairedChar("(") + ast.parameters.map(function (x, i) { return recurse(["parameters", i]); }).join("," + style.spaceInExpression) + createPairedChar(")");
            break;
        case "FORMAL_PARAMETER":
            result += ast.modifiers.map(function (x, i) { return recurse(["modifiers", i]) }).join("") +
                recurse("typeType") + " " + recurse("id");
            break;
        case "Block":
            result += indent(
                createPairedChar("{") +
                (ast.statements.length ? ("\n" +
                    ast.statements.map(function (x, i) { return recurse(["statements", i]) + "\n" }).join(""))
                    : "") +
                createPairedChar("}")
                , style.indentBy, style.javaBracketsStyle, true);
            break;
        case "ExpressionStatement":
            result += recurse("expression") + ";";
            break;
        case "PostfixExpression":
            result += recurse("operand") +
                recurse({type: "Operator", operator: ast.operator});
            break;
        case "InfixExpression":
            result += recurse("leftOperand") + style.spaceInExpression +
                recurse({type: "Operator", operator: ast.operator}) + style.spaceInExpression +
                (ast.rightOperand ? recurse("rightOperand") : "") ;
            break;
        case "Assignment":
            result += recurse("leftHandSide") + style.spaceInExpression +
                recurse({type: "Operator", operator: ast.operator}) + style.spaceInExpression +
                (ast.rightHandSide ? recurse("rightHandSide") : "") ;
            break;
        case "Operator":
            result += ast.operator;
            break;
        case "SEMI_COLON_STATEMENT":
            result += ";"
            break;
        case "ReturnStatement":
            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">return</span> " : "return ") + recurse("expression") + ";";
            break;
        case "ThrowStatement":
            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">throw</span> " : "throw ") + recurse("expression") + ";";
            break;
        case "ForStatement":
            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">for</span>" : "for") +
                (style.spaceAfterStatement) + createPairedChar("(") + 
                ast.initializers.map((x,i) => recurse(["initializers", i])).join("," + style.spaceInExpression) + ";" + style.spaceInExpression +
                recurse("expression") + ";" + style.spaceInExpression +
                ast.updaters.map((x,i) => recurse(["updaters", i])).join("," + style.spaceInExpression) +
                createPairedChar(")") +
                bracketTypes[+!!style.javaBracketsStyle] +
                recurse(conditionallyRemoveBracketsFromSingleLineBlocks(ast.body), "body");
            break;
        case "BASIC_FOR_CONTROL":
            result += recurse("forInit") + ";" + style.spaceInExpression +
                recurse("expression") + ";" + style.spaceInExpression +
                recurse("expressionList");
            break;
        case "EnhancedForStatement":
            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">for</span>" : "for") +
                (style.spaceAfterStatement) + createPairedChar("(") + 
                recurse("parameter") + style.spaceInExpression + 
                recurse({ type: "Operator", operator: ":" }) + style.spaceInExpression + 
                recurse("expression") +
                createPairedChar(")") +
                bracketTypes[+!!style.javaBracketsStyle] +
                recurse(conditionallyRemoveBracketsFromSingleLineBlocks(ast.body), "body");
            break;
        case "EXPRESSION_LIST":
        case "CATCH_TYPE":
            result += ast.list.map(function (x, i) { return recurse(["list", i]) }).join("," + style.spaceInExpression);
            break;
        case "QUALIFIED_EXPRESSION":
            var varScope;

            if (!style.dontRegisterVariables) {
                var expressionName = ast.expression.value || ast.expression.type.toLowerCase();
                varScope = getVariableScope(parentScope, expressionName);
            }
            //override the scope to go to the indicated area!            
            result += recurse("expression") + "." + recurse("rest", "rest", varScope && varScope.scope);
            break;
        case "MethodInvocation":

            result += (ast.expression ? recurse("expression") + "." : "") +
                recurse("name") + createPairedChar("(") +
                ast.arguments.map((x,i)=>recurse(["arguments",i])).join(","+style.spaceInExpression) + createPairedChar(")");
            break;
        case "IfStatement":
            function isSingleLinesAllTheWayDown(st) {
                if (!st || !st.thenStatement) return true;
                else return st.thenStatement.type != "Block" && isSingleLinesAllTheWayDown(st.elseStatement);
            }
            var isSingleLines = isSingleLinesAllTheWayDown(ast);

            var indenter = isSingleLines ? "" : style.indentBy;

            var lineSep = bracketTypes[+!!style.javaBracketsStyle];
            if (style.singleLineBlockBrackets == "line" || ast.thenStatement.type != "Block") lineSep = " ";

            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">if</span>" : "if") +
                style.spaceAfterStatement + createPairedChar("(") + recurse("expression") + createPairedChar(")") +
                lineSep +
                recurse(conditionallyRemoveBracketsFromSingleLineBlocks(ast.thenStatement), "thenStatement") +
                (ast.elseStatement ?
                    (style.singleLineBlockBrackets == "block" || (ast.body.type == "Block" && ast.thenStatement.statements.length > 1) ? style.ifElseNewline : "\n") + //if it's a single-line, then the else separator is *always* \n
                    (style.colorize ? "<span class=\"hlast hlast-keyword\">else</span>" : "else") +
                    lineSep + recurse(conditionallyRemoveBracketsFromSingleLineBlocks(ast.elseStatement), "elseStatement")
                    : "");
            break;
        case "TryStatement":
            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">try</span>" : "try") +
                bracketTypes[+!!style.javaBracketsStyle] +
                recurse("body") +
                style.ifElseNewline +
                (ast.catchClauses.map(function (x, i) { return recurse(["catchClauses", i]) }).join(style.ifElseNewline)) +
                recurse("finally");
            break;
        case "CatchClause":
            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">catch</span>" : "catch") +
                createPairedChar("(") + recurse("exception") + createPairedChar(")") +
                bracketTypes[+!!style.javaBracketsStyle] +
                indent(recurse("body"), style.indentBy, style.javaBracketsStyle, true);
            break;
        case "ConditionalExpression":
            result += recurse("expression") + style.spaceInExpression + recurse({type: "Operator", operator: "?"}) +
            style.spaceInExpression + recurse("thenExpression") + style.spaceInExpression + recurse({type: "Operator", operator: ":"}) +
            style.spaceInExpression + recurse("elseExpression");
            break;
        case "PrimitiveType":
            result += ast.primitiveTypeCode;
            break;
        case "ThisExpression":
            if(ast.qualifier != null) throw "this.qualifier isn't null!! ! !!";

            result += style.colorize ? "<span class=\"hlast hlast-keyword\">this</span>" : "this";
            break;
        case "ConstructorInvocation":
            console.log("Use of ConstructorInvocation. Make sure it's a statement!");

            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">this</span>" : "this") +
                createPairedChar("(") + 
                ast.arguments.map((x,i)=>recurse(["arguments",i])).join("," + style.spaceInExpression) + createPairedChar(")") + ";";

            break;
        case "SuperConstructorInvocation":
            if(ast.expression || ast.typeArguments.length) throw "unexpected expression or typeArguments in a SuperConstructorInvocation!";

            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">super</span>" : "super") +
                style.spaceAfterStatement +
                (ast.arguments ? createPairedChar("(") + 
                    ast.arguments.map((x,i)=>recurse(["arguments",i])).join("," + style.spaceInExpression)
                    + createPairedChar(")") 
                : "");
            break;
        case "SuperMethodInvocation":
            if(ast.qualifier) throw "unknown 'qualifier' in a SuperMethodInvocation";

            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">super</span>" : "super") + "." + recurse("name") +
            createPairedChar("(") + 
            ast.arguments.map((x,i)=>recurse(["arguments",i])).join("," + style.spaceInExpression) + createPairedChar(")");

            break;
        case "NullLiteral":
            result += "null";
            break;
        case "ContinueStatement":
            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">continue</span>" : "continue") +
                (ast.label ? " " + recurse("label") : "") + ";";
            break;
        case "ArrayAccess":
            result += recurse("array") + createPairedChar("[") + recurse("index") + createPairedChar("]");
            break;
        case "DIMENSION":
            result += style.colorize ? createPairedChar("[") + recurse("expression") + createPairedChar("]") : `[${recurse("expression")}]`;
            break;
        case "CLASS_OR_INTERFACE_TYPE_ELEMENT":
            result += recurse("name") +
                (ast.typeArguments ? recurse("typeArguments") : "") +
                (ast.dimensions ? ast.dimensions.map(function (x, i) { return recurse(["dimensions", i]); }).join("") : "");
            break;
        case "SimpleType":
            result += recurse("name");
            break;
        case "ArrayType":
            result += recurse("componentType") + createPairedChar("[") + createPairedChar("]");
            break;
        case "ARRAY_CREATOR_REST":
            result += ast.dimensions.map(function (x, i) { return recurse(["dimensions", i]); }).join("") +
                (ast.arrayInitializer ? style.spaceInExpression + recurse("arrayInitializer") : "");
            break;
        case "ArrayInitializer":
            result += createPairedChar("{") + style.spaceAfterStatement +
                ast.expressions.map(function (x, i) { return recurse(["expressions", i]); }).join("," + style.spaceInExpression) +
                style.spaceAfterStatement + createPairedChar("}");
            break;
        case "ImportDeclaration":
            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">import</span> " : "import ") +
                (ast.static ? "static " : "") +
                recurse("name") +
                (ast.onDemand ? ".*" : "");
            break;
        case "CLASS_OR_INTERFACE_TYPE":
            result += ast.elements.map(function (x, i) { return recurse(["elements", i]); }).join(".");
            break;
        case "ClassInstanceCreation":
            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">new</span> " : "new ") + recurse("typeType") + 
                (ast.typeArguments.length ? 
                    createPairedChar("<") + ast.typeArguments.map((x,i)=>recurse(["typeArguments", i])).join("," + style.spaceInExpression) + createPairedChar(">") 
                    : "") +
                createPairedChar("(") + ast.arguments.map((x,i)=>recurse(["arguments", i])).join("," + style.spaceInExpression) + createPairedChar(")");
            break;
        case "ParameterizedType":
            result += recurse("typeType") + createPairedChar("<") + 
                (ast.typeArguments.length ? ast.typeArguments.map((x,i)=>recurse(["typeArguments", i])).join("," + style.spaceInExpression) : "") + 
                createPairedChar(">");
            break;
        case "ArrayCreation":
            if(ast.initializer && ast.dimensions.length > 0) {
                console.log(ast);
                throw "unexpected initializer";
            }
            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">new</span> " : "new ") + recurse(["typeType", "componentType"]) + 
                (ast.initializer ? " " + recurse("initializer") : "") +
                ast.dimensions.map((x,i)=>createPairedChar("[") + recurse(["dimensions", i]) + createPairedChar("]"));
            break;
        case "IDENTIFIER_NAME":
            result += ast.elements.map(function (x, i) { return recurse(["elements", i]) }).join("," + style.spaceAfterStatement);
            break;
        case "IDENTIFIER_NAME_ELEMENT":
            result += recurse("id") + style.spaceAfterStatement +
                (ast.typeArguments === undefined ? "" : recurse("typeArguments"));
            break;
        case "TYPE_ARGUMENT":
            result += recurse("argument") +
                (ast.extends ? (style.colorize ? " <span class=\"hlast hlast-keyword\">extends</span> " : " extends ") + recurse("extends") : "")
            break;
        case "CLASS_CREATOR_REST":
            result += createPairedChar("(") + recurse("arguments") + createPairedChar(")");
            break;
        case "CastExpression":
            result += createPairedChar("(") + recurse("typeType") + createPairedChar(")") + recurse("expression");
            break;
        case "PrefixExpression":
            result += ast.operator + recurse("operand");
            break;
        case "ParenthesizedExpression":
            result += createPairedChar("(") + recurse("expression") + createPairedChar(")");
            break;
        case "comment":
            result += indent(undent(ast.value), " ", true, false);
            if (ast.value.split("\n").length > 1 || ast.value.startsWith("//")) result += "\n";
            break;
        case "TYPE_ARGUMENT":
            result += recurse("argument") +
                (ast.extends ? " " + ((style.colorize ? "<span class=\"hlast hlast-keyword\">extends</span> " : "extends ") + recurse("extends")) : "")
            break;
        case "WhileStatement":
            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">while</span>" : "while") +
                style.spaceAfterStatement + createPairedChar("(") + recurse("expression") + createPairedChar(")") +
                bracketTypes[+!!style.javaBracketsStyle] +
                recurse(conditionallyRemoveBracketsFromSingleLineBlocks(ast.body), "body");
            break;
        case "BreakStatement":
            result += (style.colorize ? "<span class=\"hlast hlast-keyword\">break</span>" : "break") +
                (ast.label ? (" " + recurse("label")) : "") +
                ";";
            break;
        case "MarkerAnnotation":
            result += "@" + recurse("typeName") + "\n";
            break;
        default:
            console.log("unknown type " + ast.type + " at " + address.join("."));
            console.log(ast);
            result += "";
    }

    if (style.isDense) result = result.trim();
    if (ast.followedEmptyLine) result += "\n";
    if (style.isDense) result = result.replace(/\n/g, "").trim();



    var formattedRes = result;

    //only colorize single-line things bc that way it won't get messed up upon table-ifying
    if (style.colorize) {
        formattedRes = result.split("\n").map(function (line) {
            return `<span class="hlast hlast-${camelKebab(ast.type || ast.ast_type || "")}${generateDescribingClasses(result, siblingIndex)}" data-address=${address.join(".")} data-annotation-connector-id="0${annotationConnectorId.replace(/,$/, "")}" data-nodepath="${nodePath.map(function (x, i) { return camelKebab(x.type || x.ast_type || "") }).join(" ")}">${(line)}</span>`;
        }).join("\n");
    }

    if (ast.comments && !style.isDense && !style.removeComments) {
        for (var i = ast.comments.length - 1; i >= 0; i--) {
            var formattedVal = recurse(["comments", i]);
            if (ast.comments[i].leading) formattedRes = formattedVal + formattedRes;
            else formattedRes += formattedVal;
        }
    }

    if (address.length == 0) finalActions();

    return formattedRes;
}

function getAstJavaEquivType(ast, scope) {

}

function finalActions() {
    //console.info(globalVarRegistry);
}

function generateDescribingClasses(str, idx) {
    var classes = "";
    classes += " sibling-" + idx;
    if (/^[A-Z][a-z]/.test(str)) classes += " capitalized";
    if (/^[A-Z_]+$/.test(str)) classes += " uppercase";
    if (str.length == 1) classes += " singlechar";
    return classes;
}

/**
 * Convert a camelCase (or CamelCase) string to kebab-case
 * @param {String} camel The camelCase string to be converted
 */
function camelKebab(camel) {
    var r = "", lengthGoneWithoutNewWord = 0;
    for(var i = 0; i < camel.length; i++) {
        var nowChar = camel.charAt(i);
        var nowCharLower = nowChar.toLowerCase();

        var isUpperCase = nowChar != nowCharLower;

        //add a new word
        if(isUpperCase && lengthGoneWithoutNewWord > 0) {
            lengthGoneWithoutNewWord = 0;
            r += "-" + nowCharLower;
        } else {
            //or just add the next letter
            r += nowCharLower;
            lengthGoneWithoutNewWord++;
        }
    }
    return r;
}

function indent(indentText, indentBy, dontIndentFirst, dontIndentLast) {
    var lines = indentText.split("\n");

    for (var i = 1; i < lines.length - +dontIndentLast; i++) lines[i] = indentBy + lines[i];
    return lines.join("\n");
}

function undent(text) {
    return text.split("\n").map(function (x) { return x.trim() }).join("\n");
}

function getScopeComponent(ast, address) {
    switch (ast.type) {
        case "TypeDeclaration":
            return "$" + ast.name.identifier;
        case "CONSTRUCTOR_DECLARATION":
            return "[constructor]" + "(" + ast.parameters.parameters.map(function (x) { return astToString(x.typeType, {}); }).join(",") + ")";
        case "MethodDeclaration":
            return "." +
                astToString(ast.name) + "(" + ast.parameters.map(function (x) { return astToString(x.typeType, {}); }).join(",") + ")";
        case "ForStatement":
        case "IfStatement":
        case "WhileStatement":
            var statementInsideBodyIndex = "";
            //start after type defs
            for (i = address.indexOf("statements"); i < address.length - 1; i++) {
                if (typeof address[i] === "number") statementInsideBodyIndex += "/" + address[i];
            }
            if (statementInsideBodyIndex == "") return "";
            else return statementInsideBodyIndex;
        default:
            return "";
    }
}

function createScope(newScope) {
    newScope = newScope.filter(function (x) { return x != "" });

    if (newScope.length == 0) return false;

    var target = globalVarRegistry;
    for (var i = 0; i < newScope.length; i++) {
        if (!target.children) target.children = {};
        if (!target.children[newScope[i]]) target.children[newScope[i]] = {};
        target = target.children[newScope[i]];
    }
}

function registerVariable(scope, varName, typeType, type) {
    scope = scope.filter(function (x) { return x != "" });

    var address = "", scopeKey = scope.join("");

    if (type == "class") address = scopeKey;
    else address = scopeKey + "." + varName;

    var target = globalVarRegistry;
    for (var i = 0; i < scope.length; i++) {
        if (!target.children[scope[i]]) target.children[scope[i]] = {};
        target = target.children[scope[i]];
    }

    if (type != "class") {
        if (!target.vars) target.vars = {};
        if (!target.vars[varName]) target.vars[varName] = {};
        target = target.vars[varName];
    }

    Object.assign(target, {
        name: varName,
        address: address,
        scope: scope,
        scopeKey: scope.join(""),
        typeType: typeType,
        type: type || "variable",
        privacy: "public" //TODO: implement
    });

    return target;
}

function getClassScope(currentScope, className) {
    for (var i = currentScope.length; i > 0; i--) {
        var scannedScope = currentScope.slice(0, i);

        var target = globalVarRegistry;
        for (var j = 0; j < scannedScope.length; j++) {
            target = target.children[scannedScope[j]];
        }

        if (!target.children) continue;
        console.info(target, className)
        if (target.children["$" + className]) return target.children["$" + className];
    }
    return null;
}

function getVariableScope(currentScope, varName) {
    currentScope = currentScope.filter(function (x) { return x != "" });

    for (var i = currentScope.length; i > 0; i--) {
        var scannedScope = currentScope.slice(0, i);

        var target = globalVarRegistry;
        for (var j = 0; j < scannedScope.length; j++) {
            target = target.children[scannedScope[j]];
        }

        if (!target) console.warn("unknown scope", currentScope, varName, globalVarRegistry);

        if (!target.vars) continue;
        if (varName) target = target.vars[varName]

        if (typeof target === "object") return target;
        else if (varName === "this" && currentScope[i - 1].startsWith("$")) return {
            scope: scannedScope,
            scopeKey: scannedScope.join(""),
            name: currentScope[i - 1],
            type: "class"
        }
    }
    return null;
}

function encodeCharacterEntities(str) {
    if (!str) return "";
    return str.replace(/&/g, "&amp;")
        .replace(/"/g, "&quot;")
        .replace(/'/g, "&apos;")
        .replace(/</g, "&lt;")
        .replace(/>/g, "&gt;");
}