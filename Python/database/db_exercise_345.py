"""
데이터베이스 연습 #345
SQLite CRUD 실습
"""
import sqlite3

def setup_345():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_345 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_345_0', 421.47, 'B'), ('item_345_1', 493.81, 'A'), ('item_345_2', 569.74, 'A'), ('item_345_3', 299.17, 'B'), ('item_345_4', 232.59, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_345 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_345()
    cursor = conn.cursor()

    print(f"DB 연습 #345")
    cursor.execute('SELECT * FROM items_345')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_345 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
