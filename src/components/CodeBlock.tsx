'use client';

import { useState } from 'react';
import { Copy, Check } from 'lucide-react';

/**
 * Token-based Python Syntax Highlighter
 * Tokenizes first, then renders - avoids regex conflicts
 */
type TokenType = 'keyword' | 'string' | 'comment' | 'builtin' | 'self' | 'rclpy' | 'node' | 'constant' | 'text';

interface Token {
  type: TokenType;
  value: string;
}

const KEYWORDS = new Set(['import', 'from', 'class', 'def', 'return', 'if', 'else', 'elif', 'try', 'except', 'finally', 'pass', 'while', 'for', 'in', 'as', 'with', 'super', 'and', 'or', 'not', 'is', 'lambda', 'yield', 'raise', 'break', 'continue', 'global', 'nonlocal', 'assert', 'del']);
const CONSTANTS = new Set(['None', 'True', 'False']);

const tokenizeLine = (line: string): Token[] => {
  const tokens: Token[] = [];
  let i = 0;
  
  while (i < line.length) {
    // Check for comment
    if (line[i] === '#') {
      tokens.push({ type: 'comment', value: line.slice(i) });
      break;
    }
    
    // Check for strings (single or double quotes)
    if (line[i] === '"' || line[i] === "'") {
      const quote = line[i];
      let j = i + 1;
      while (j < line.length && (line[j] !== quote || line[j-1] === '\\')) {
        j++;
      }
      tokens.push({ type: 'string', value: line.slice(i, j + 1) });
      i = j + 1;
      continue;
    }
    
    // Check for words (identifiers/keywords)
    if (/[a-zA-Z_]/.test(line[i])) {
      let j = i;
      while (j < line.length && /[a-zA-Z0-9_]/.test(line[j])) {
        j++;
      }
      const word = line.slice(i, j);
      
      if (KEYWORDS.has(word)) {
        tokens.push({ type: 'keyword', value: word });
      } else if (CONSTANTS.has(word)) {
        tokens.push({ type: 'constant', value: word });
      } else if (word === 'self') {
        tokens.push({ type: 'self', value: word });
      } else if (word === 'rclpy') {
        tokens.push({ type: 'rclpy', value: word });
      } else if (word === 'Node') {
        tokens.push({ type: 'node', value: word });
      } else {
        tokens.push({ type: 'text', value: word });
      }
      i = j;
      continue;
    }
    
    // Any other character
    tokens.push({ type: 'text', value: line[i] });
    i++;
  }
  
  return tokens;
};

const tokenToElement = (token: Token, index: number): React.ReactNode => {
  const styles: Record<TokenType, string> = {
    keyword: 'text-purple-600 font-bold',
    string: 'text-green-600',
    comment: 'text-zinc-400 italic',
    builtin: 'text-cyan-600',
    self: 'text-blue-600 italic',
    rclpy: 'text-blue-500 font-bold',
    node: 'text-yellow-600 font-bold',
    constant: 'text-orange-500',
    text: '',
  };
  
  const className = styles[token.type];
  
  if (className) {
    return <span key={index} className={className}>{token.value}</span>;
  }
  return <span key={index}>{token.value}</span>;
};

const highlightPython = (code: string): React.ReactNode[] => {
  return code.split('\n').map((line, lineIndex) => {
    const tokens = tokenizeLine(line);
    return (
      <div key={lineIndex}>
        {tokens.length > 0 ? tokens.map((token, tokenIndex) => tokenToElement(token, tokenIndex)) : ' '}
      </div>
    );
  });
};

interface CodeBlockProps {
  code: string;
  language?: string;
  filename: string;
}

export default function CodeBlock({ code, filename }: CodeBlockProps) {
  const [copied, setCopied] = useState(false);

  const handleCopy = () => {
    navigator.clipboard.writeText(code);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <div className="my-10 rounded-xl overflow-hidden border border-zinc-200 bg-white shadow-xl">
      <div className="flex items-center justify-between px-4 py-3 bg-zinc-50 border-b border-zinc-100">
        <div className="flex items-center space-x-2">
          <div className="flex space-x-1.5">
            <div className="w-3 h-3 rounded-full bg-red-400"></div>
            <div className="w-3 h-3 rounded-full bg-yellow-400"></div>
            <div className="w-3 h-3 rounded-full bg-green-400"></div>
          </div>
          <span className="ml-3 text-xs font-mono text-zinc-500 font-medium">{filename}</span>
        </div>
        <button 
          onClick={handleCopy}
          className="flex items-center space-x-1.5 px-2 py-1 rounded hover:bg-zinc-200 transition-colors text-xs text-zinc-500 font-medium cursor-pointer"
        >
          {copied ? <Check size={14} className="text-green-600" /> : <Copy size={14} />}
          <span>{copied ? 'Copied' : 'Copy'}</span>
        </button>
      </div>
      <div className="p-6 overflow-x-auto bg-[#fbfbfb]">
        <pre className="text-sm font-mono text-zinc-800 leading-7">
          <code>{highlightPython(code)}</code>
        </pre>
      </div>
    </div>
  );
}
