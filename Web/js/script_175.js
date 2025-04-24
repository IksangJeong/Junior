// JavaScript 연습 #175

// 배열 처리 함수
function processData_175(arr) {
    const sorted = [...arr].sort((a, b) => a - b);
    const sum = arr.reduce((acc, val) => acc + val, 0);
    const avg = sum / arr.length;
    const max = Math.max(...arr);
    const min = Math.min(...arr);

    return {
        sorted,
        sum,
        avg: avg.toFixed(2),
        max,
        min,
        count: arr.length,
    };
}

// 문자열 유틸리티
function formatString_175(str) {
    return str
        .trim()
        .split(/\s+/)
        .map(word => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
        .join(' ');
}

// 테스트
const testData = [25, 75, 25, 75, 75];
console.log("연습 #175");
console.log("입력:", testData);
console.log("결과:", processData_175(testData));
console.log("포맷:", formatString_175("hello world javascript"));
