// JavaScript 연습 #157

// 배열 처리 함수
function processData_157(arr) {
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
function formatString_157(str) {
    return str
        .trim()
        .split(/\s+/)
        .map(word => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
        .join(' ');
}

// 테스트
const testData = [99, 41, 11, 9, 37];
console.log("연습 #157");
console.log("입력:", testData);
console.log("결과:", processData_157(testData));
console.log("포맷:", formatString_157("hello world javascript"));
